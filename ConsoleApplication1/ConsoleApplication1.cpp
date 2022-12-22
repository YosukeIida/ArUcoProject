
//------------------------------------------------------------------------------
//  2022/12/07作成
//
//
//  VC++ディレクトリ / インクルードディレクトリに
//      C:\opencv347\build\install\include
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\include
//
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\include        (自宅用 kinect sdk 1.4.1のとき使用)
//  を追加する．
//
//  VC++ディレクトリ / ライブラリディレクトリに
//      C:\opencv347\build\install\x64\vc16\lib
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\windows-desktop\amd64\release\lib
//
//      C:\opencv347\build\install\lib      (自宅用 opencv347)
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\lib          (自宅用 kinect sdk 1.4.1)
//
//------------------------------------------------------------------------------


#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <k4a/k4a.h>
#include <k4a/k4atypes.h>

#include "kinectdevice.h"

#define _USE_MATH_DEFINES
#include <math.h>



# define COLOR_WIDTH        1920            // カラーセンサ の横幅
# define COLOR_HEIGHT       1080            // カラーセンサ の縦幅
# define MARKER_LENGTH      0.018           // マーカの1辺の長さ [m]


// Visual C++でコンパイルするときにリンクするライブラリファイル
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui347.lib")
#pragma comment(lib, "opencv_calib3d347.lib")
#pragma comment(lib, "opencv_aruco347.lib")





// capture から color_imageを取得
void get_color_image_data(k4a_image_t* color_image_handle, uint8_t** color_image_buffer) {

    *color_image_buffer = k4a_image_get_buffer(*color_image_handle);
}


int main()
{
    cv::Mat rgbaImg;                // カラーセンサの画像
    cv::Mat colorImg;               // arucoで使用  4chRGBA -> 3chRGB


        // デバイスで取得した画像は k4a_device_get_capture() によって返される k4a_capture_t オブジェクトを通して取得
        // k4a_image_t は画像データと関連するメタデータを管理する
    k4a_capture_t capture;              // kinectのキャプチャハンドル
                                        // ほぼ同時にデバイスで記録したColor，Depth，Irなどの一連のデータを表す．

        // カラーイメージ
    k4a_image_t color_image_handle;     // キャプチャのカラーセンサのハンドル

    uint8_t* color_image_buffer;        // カラーイメージのデータのポインタ


    cv::Mat camera_matrix(3, 3, CV_64FC1);
    camera_matrix.at<double>(0, 0) = 909.98956298828125000000;      // fx
    camera_matrix.at<double>(0, 1) = 0.0;                           // 0.0
    camera_matrix.at<double>(0, 2) = 961.07861328125000000000;      // cx
    camera_matrix.at<double>(1, 0) = 0.0;                           // 0.0
    camera_matrix.at<double>(1, 1) = 909.63812255859375000000;      // fy
    camera_matrix.at<double>(1, 2) = 553.43408203125000000000;      // cy
    camera_matrix.at<double>(2, 0) = 0.0;                           // 0.0
    camera_matrix.at<double>(2, 1) = 0.0;                           // 0.0
    camera_matrix.at<double>(2, 2) = 1.0;                           // 1.0

    cv::Mat dist_coeffs(1, 5, CV_64FC1);
    dist_coeffs.at<double>(0, 0) = 0.46717128157615661621;          // k1
    dist_coeffs.at<double>(0, 1) = -2.45866727828979492188;         // k2
    dist_coeffs.at<double>(0, 2) = 0.00136364088393747807;          // p1
    dist_coeffs.at<double>(0, 3) = -0.00006751885666744784;          // p2
    dist_coeffs.at<double>(0, 4) = 1.37056386470794677734;         // k3


    std::cout << camera_matrix << std::endl;
    std::cout << dist_coeffs << std::endl;







    // Create Marker Dictionary, Type of marker : 4x4, 1000
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_1000;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);
    
    // 検出したマーカーのIDを格納するベクター
    std::vector<int> marker_ids;
    
    //検出したマーカーのコーナー座標とリジェクト座標を格納する
    std::vector<std::vector<cv::Point2f>> marker_corners, rejectedCandidates;
    
    int sum_ids = 0;
    cv::Mat markerImage;
    
    


    // detectorパラメータを初期化する
    



    // インスタンスの生成 生成時にコンストラクタが呼び出される
// コンストラクタでデバイスのオープン, カメラ構成設定, カメラのスタートを行う
    KinectDevice kinectdevice;






    try
    {
        while (true) {
            // キャプチャが成功しているかどうかを調べる
            switch (k4a_device_get_capture(kinectdevice.device, &capture, 1000)) {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;      // メインループ抜ける
            case K4A_WAIT_RESULT_TIMEOUT:
                std::cout << "キャプチャタイムアウト" << std::endl;
                continue;   // もう一度
            case K4A_WAIT_RESULT_FAILED:
                throw std::runtime_error("キャプチャに失敗しました");
            }

            // キャプチャハンドルからカラーイメージのハンドルを取得する
            color_image_handle = k4a_capture_get_color_image(capture);
            // キャプチャハンドルからデプスイメージのハンドルを取得する

            // カラーイメージのハンドルから画像のデータ，高さ，幅を取得する
            if (color_image_handle) {
                get_color_image_data(&color_image_handle, &color_image_buffer);

                // カラーセンサのデータをRGBA画像に変換する
                rgbaImg = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);      //4ch RGBA画像
                rgbaImg.data = color_image_buffer;

                colorImg = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC3);     // 3ch RGB画像
                cv::cvtColor(rgbaImg, colorImg, CV_RGBA2RGB);

                cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
                // サブピクセル化ON
                parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

                // arucoマーカを検出する
                cv::aruco::detectMarkers(colorImg, dictionary, marker_corners, marker_ids, parameters, rejectedCandidates);
                
                //// cv::outputarray を使って vector<int> を vector<cv::Mat>に変換して表示
                //cv::OutputArray marker_ids_outary = marker_ids;
                //std::vector<cv::Mat> show_marker_ids;
                //marker_ids_outary.getMatVector(show_marker_ids);

                //for (auto show_marker_ids : show_marker_ids) {
                //    std::cout << show_marker_ids << std::endl;
                //}

                std::cout << "id:";
                for (size_t num = 0; num < marker_ids.size(); num++) {
                    std::cout << marker_ids[num] << ",";
                }
                std::cout << std::endl;

                for (size_t num = 0; num < marker_ids.size(); num++) {
                    std::cout << "[" << marker_ids[num] << "]";
                    std::cout << "corner:";
                    std::cout << marker_corners[num] << std::endl;
                }

                std::cout << std::endl << std::endl;

                if (marker_ids.size() > 0) {
                    
                    // 検出したマーカを可視化
                    cv::aruco::drawDetectedMarkers(colorImg, marker_corners, marker_ids);
                    std::vector<cv::Vec3d> rvecs, tvecs;

                    // マーカのrvec, tvecを求める
                    cv::aruco::estimatePoseSingleMarkers(marker_corners, MARKER_LENGTH, camera_matrix, dist_coeffs, rvecs, tvecs);
                    for (int i = 0; i < marker_ids.size(); i++) {
                        std::cout << "tvecs:" << tvecs[i] * 1000 << std::endl;      // 単位[mm]
                        std::cout << "rvecs:" << rvecs[i] << std::endl;
                        cv::aruco::drawAxis(colorImg, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 5);
                    }
                }




                


                //cv::imshow("rgbaImg", rgbaImg);
                cv::imshow("colorImg", colorImg);

                k4a_image_release(color_image_handle);
                k4a_capture_release(capture);
            }


            // キー入力 "q" でプログラムを終了する
            const int key = cv::waitKey(30);
            if (key == 'q') {
                break;
            }


        }
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}



