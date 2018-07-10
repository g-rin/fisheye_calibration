#include <iostream>
#include <sstream>
#include <string>
#include <dirent.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <popt.h>

void printMatrix( const std::string& title,
                  const cv::Mat& matrix );
void printPoints( const std::vector< cv::Point2f >& points );
std::vector< std::string > getInputImageList( const std::string& dirname );

int main( int argc, char**argv ) {
    // NOTE:
    // the chessboard pattern is 10x7 squares
    // chessboard's square size is 25 mm.
    // const double distance = 680.0; // 68 cm form the camera to the floor
    // const double real_offset = 5.0; // each

    char* image_dir;
    int board_rows = 0;
    int board_cols = 0;
    float square_size = 0;

    static struct poptOption options[] = {
        { "image_dir", 'i', POPT_ARG_STRING, &image_dir, 0, "Directory containg images", "STR" },
        { "board_rows", 'r', POPT_ARG_INT, &board_rows, 0, "Checkerboard row count", "NUM" },
        { "board_cols", 'c', POPT_ARG_INT, &board_cols, 0, "Checkerboard column count", "NUM" },
        { "square_size",'s',POPT_ARG_FLOAT, &square_size, 0, "Checkerboard square size","NUM" },
        POPT_AUTOHELP
        { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    poptContext arguments;
    arguments = poptGetContext( NULL, argc, (const char**)argv, options, 0 );
    while( 0 <= poptGetNextOpt( arguments ) ) {}
    poptFreeContext( arguments );

    cv::Size board_size( board_rows-1, board_cols-1 );


    std::vector<  std::vector< cv::Point2f > > found_corners;
    cv::Size image_size;

    const auto filelist = getInputImageList( image_dir );
    for( const auto& filename : filelist ) {
        cv::Mat image = cv::imread( filename, cv::IMREAD_COLOR );
        image_size = image.size();

        std::vector< cv::Point2f > points;
        const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
        const bool found = findChessboardCorners( image,
                                                  board_size,
                                                  points,
                                                  flags );
        if( found ) {
                cv::Mat gray;
                cvtColor( image,
                          gray,
                          cv::COLOR_BGR2GRAY);

                cv::TermCriteria criteria;
                criteria.type = cv::TermCriteria::EPS + cv::TermCriteria::COUNT;
                criteria.maxCount = 30;
                criteria.epsilon = 0.1;

                cv::cornerSubPix( gray,            // image
                                  points,          // corners
                                  cv::Size(11,11), // winSize
                                  cv::Size(-1,-1), // zeroZone
                                  criteria );

                printf( "Point of image '%s'\n", filename.c_str() );
                printPoints( points );

                found_corners.push_back( points );
                drawChessboardCorners( image,
                                       board_size,
                                       cv::Mat( points ),
                                       true );
        }

        cv::imshow("Input", image);
        cv::waitKey( 50 );
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    std::vector< std::vector< cv::Point3f > > objectPoints(1);
    for( int row = 0; row < board_size.height; ++row ) {
        for( int column = 0; column < board_size.width; ++column ) {
            const auto point = cv::Point3f( column * square_size,
                                            row * square_size,
                                            0 );
            objectPoints[0].push_back( point );
        }
    }
    objectPoints.resize(found_corners.size(),objectPoints[0]);

    const int flags =
//            cv::fisheye::CALIB_USE_INTRINSIC_GUESS |
//            cv::fisheye::CALIB_FIX_K1 |
//            cv::fisheye::CALIB_FIX_K2 |
//            cv::fisheye::CALIB_FIX_K3 |
//            cv::fisheye::CALIB_FIX_INTRINSIC |
                      cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                      cv::fisheye::CALIB_CHECK_COND |
                      cv::fisheye::CALIB_FIX_SKEW |
                      cv::fisheye::CALIB_FIX_K4 |
                      cv::fisheye::CALIB_FIX_PRINCIPAL_POINT
            ;

    cv::Mat rvecs, tvecs;
    const double rms = cv::fisheye::calibrate( objectPoints,
                                               found_corners,
                                               image_size,
                                               camera_matrix,
                                               distortion_coeffs,
                                               rvecs,
                                               tvecs,
                                               flags );
    printMatrix( "camera", camera_matrix );
    printMatrix( "coeffs", distortion_coeffs );

    std::cout << "Re-projection error reported by cv::fisheye::calibrate: "<< rms << std::endl;

    cv::Mat view, rview, map1, map2;

    cv::Mat newCamMat;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
                camera_matrix,
                distortion_coeffs,
                image_size,
                cv::Matx33d::eye(),
                newCamMat,
                1 );

    cv::fisheye::initUndistortRectifyMap(
                camera_matrix,
                distortion_coeffs,
                cv::Matx33d::eye(),
                newCamMat,
                image_size,
                CV_16SC2,
                map1,
                map2 );

    for( const auto& filename : filelist ) {
        cv::Mat view = cv::imread( filename, cv::IMREAD_COLOR );
        remap(view, rview, map1, map2, cv::INTER_LINEAR);
        imshow("Output", rview);
        const int ESC_KEY = 27;
        if( ESC_KEY == cv::waitKey() )
            break;
    }

    return 0;
}

void printMatrix( const std::string& title,
                  const cv::Mat& matrix )
{
    const auto row_count = matrix.rows;
    const auto col_count = matrix.cols;
    std::cout << "Matrix " << title << std::endl;
    if( ( row_count > 0 ) && ( col_count ) ) {
        for( int row = 0; row < row_count; ++row ) {
            for( int col = 0; col < col_count; ++col ) {
                if( col > 0 ) {
                    std::cout << ", ";
                }
                std::cout << matrix.at< double >( row, col );
            }
            std::cout << std::endl;
        }
    }
}

void printPoints( const std::vector< cv::Point2f >& points ) {
    for( const auto& point : points ) {
        std::cout << " (" << point.x << ", " << point.y << ")";
    }
    std::cout << std::endl;
}

std::vector< std::string > getInputImageList( const std::string& dirname ) {
    std::vector< std::string > result;
    DIR*const dir = opendir( dirname.c_str() );
    if( NULL != dir ) {
        struct dirent* ent = readdir( dir );
        while( NULL != ent ) {
            if( DT_REG == ent->d_type ) {
#ifdef _WIN32
                const char separator = '\\';
#else
                const char separator = '/';
#endif
                const std::string basename = ent->d_name;
                bool found = ( std::string::npos != basename.find( ".jpg" ) );
                found = found || ( std::string::npos != basename.find( ".png" ) );
                if( found ) {
                    const std::string filename = dirname + separator + ent->d_name;
                    result.push_back( filename );
                }
            }
            ent = readdir( dir );
        }
        closedir (dir);
    } else {
      std::cerr << "Unable to open directory " << dirname << std::endl;
    }

    std::sort( result.begin(), result.end() );

    return result;
}
