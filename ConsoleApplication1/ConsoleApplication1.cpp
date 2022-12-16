
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
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <k4a/k4a.h>

#include "kinect.h"

#define _USE_MATH_DEFINES
#include <math.h>



# define COLOR_WIDTH        1920            // カラーセンサ の横幅
# define COLOR_HEIGHT       1080            // カラーセンサ の縦幅


//  Visual C++でコンパイルするときにリンクするライブラリファイル
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui347.lib")
#pragma comment(lib, "opencv_aruco347.lib")


// capture から color_imageを取得
void get_color_image_data(k4a_image_t* color_image_handle, uint8_t** color_image_buffer) {

    *color_image_buffer = k4a_image_get_buffer(*color_image_handle);
}


int main()
{
    cv::Mat rgbaImg;                // カラーセンサの画像


        // デバイスで取得した画像は k4a_device_get_capture() によって返される k4a_capture_t オブジェクトを通して取得
        // k4a_image_t は画像データと関連するメタデータを管理する
    k4a_capture_t capture;              // kinectのキャプチャハンドル
                                        // ほぼ同時にデバイスで記録したColor，Depth，Irなどの一連のデータを表す．

        // カラーイメージ
    k4a_image_t color_image_handle;     // キャプチャのカラーセンサのハンドル

    uint8_t* color_image_buffer;        // カラーイメージのデータのポインタ








    // Create Marker Dictionary, Type of marker : 4x4, 1000
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_1000;
    cv::Mat markerImage;

    //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    // detectorパラメータを初期化する
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    
    // 検出したマーカーのIDを格納するベクター
    std::vector<int> marker_ids;

    //検出したマーカーのコーナー座標とリジェクト座標を格納する
    std::vector<std::vector<cv::Point2f>> marker_corners, rejectedCandidates;
    int sum_ids = 0;



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
                get_color_image_data(&color_image_handle, &color_image_height, &color_image_width, &color_image_buffer);

                // カラーセンサのデータをRGBA画像に変換する
                rgbaImg = cv::Mat(color_image_height, color_image_width, CV_8UC4);      //4ch RGBA画像
                rgbaImg.data = color_image_buffer;
                cv::imshow("rgbaImg", rgbaImg);
            }


            // キー入力 "q" でプログラムを終了する
            const int key = cv::waitKey(1);
            if (key == 'q') {
                break;
            }

            k4a_image_release(color_image_handle);
            k4a_capture_release(capture);

        }
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}



// capture から color_imageを取得
void get_color_image_data(k4a_image_t* color_image_handle, int32_t* color_image_height, int32_t* color_image_width, uint8_t** color_image_buffer) {
    *color_image_height = k4a_image_get_height_pixels(*color_image_handle);
    *color_image_width = k4a_image_get_width_pixels(*color_image_handle);

    *color_image_buffer = k4a_image_get_buffer(*color_image_handle);
}

