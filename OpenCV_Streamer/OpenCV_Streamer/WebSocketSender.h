#pragma once
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <opencv2/opencv.hpp>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <openssl/ssl.h>
#include <openssl/err.h>

#pragma comment(lib, "ws2_32")

//upd max packet size	65535 bytes
//mat 2 jpg size		about 23805 ?
//cv::Mat size			mat.step[0] * mat.rows;
//						mat.total() * mat,elemSize();
//						640 * 640 * 3 = 1,228,800 bytes
//						1,228,800 / 20 = 61,440

//MTU는 보통 1500으로 설정되어 있다.
//CMD에서 이를 변경할 수 있지만 권장되지 않는다고 한다.
//따라서 패킷을 1500에 맞추어 잘라서 보내는 로직이 필요하다.
#define PORT			5432
#define MTU				1500
#define UDP_HEADER_SIZE 28
#define INFO_SIZE		4						//카메라 번호 및 이미지 번호를 담는 임의의 사용자 정의 패킷 헤더(UDP 헤더와 별개)의 크기(바이트) - End_Flag, CameraID, ImageSegNum
#define PACKET_SIZE		MTU - UDP_HEADER_SIZE	//MTU - UPD-Header = 1500 - 28 = 1472
#define IMG_SEG_SIZE	PACKET_SIZE - INFO_SIZE
#define IMG_QUALITY		20						//jpeg 형식으로 변환할때 화질 설정
//#define SERVER_IP		"127.0.0.1"
#define SERVER_IP		"43.202.61.242"
#define SERVER_DOMAIN	"j11b209.p.ssafy.io"

#define MAX_CACHE 3

//이미지 Mat의 크기
constexpr int IMG_FULL_SIZE = 640 * 480 * 3;


/// <summary>
/// UDP를 사용하여 OpenCV의 이미지를 전송하는 클래스
/// 
/// 일반적인 컴퓨터의 MTU는 1500 바이트입니다. 이 MTU를 변경할 수 있지만 일단은 변경하지 않았다고 가정하겠습니다.
/// UDP로 패킷을 전송할 때 1500만큼 전송이 가능하지만 UDP 헤더의 크기 28바이트를 빼면 1472바이트만 전송이 가능합니다.
/// 따라서 한 UDP 패킷 전송에서 담을 수 있는 데이터는 1472바이트 또는 MTU - UDP HEADER 입니다.
/// 하지만 카메라가 여러대일 경우가 있으므로 각 패킷에는 카메라 정보가 필요합니다. 또한 이미지 세그먼트 정보 또한 필요합니다.
/// 그러므로 이러한 정보(카메라 번호, 세그먼트 번호, 마지막 세그먼트 여부)를 담을 3 바이트를 함께 전송해야합니다.
/// 그러므로 실제로 전송가능한 이미지 데이터는 한번에 MTU - UDP_HEADER - INFO_HEADER인 1469 바이트입니다.
/// 
/// MAT 형식은 OPENCV의 기본적인 이미지 형태입니다. 크기는 이미지 높이 * 너비 * 3 (RGB)로 매우 큰 형태이며 이를 그대로 전송하는 것은 비효율적입니다.
/// 또한, 이러한 MAT 형태를 그대로 사용하려면 프론트엔드에도 OPENCV가 설치되어 있어야합니다.
/// 따라서 OPENCV에서 기본 제공하는 JPEG 형식으로 변환하는 메서드를 사용하여 이를 별도의 변환없이 바로 사용할 수 있도록 합니다.
/// 압축률이나 이미지 데이터의 따라 크기가 다르지만 보통 60000 ~ 70000 바이트 정도의 크기를 가집니다.
/// 이를 앞서 계산한 전송가능한 이미지 패킷의 크기 1469 바이트로 나누고 전송합니다. sendframe_via_udp 참고
/// 
/// 서버에서는 최종적으로 얼마만큼의 패킷이 도착할지 알 수 없기 때문에 마지막 세그먼트 여부의 값을 통해 전송 중간에 전체 패킷의 크기를 알 수 있습니다.
/// 마지막 세그먼트인 경우 현재까지 받은 패킷을 프론트엔드로 전송합니다.
/// 
/// </summary>
class WebSocketSender
{
public:
	WebSocketSender();
	~WebSocketSender();

	//데이터 전송
	void sendframe_via_udp(cv::InputArray frame);

	std::vector<BYTE> mat2jpg(cv::InputArray mat);
	inline bool isconnected() { return connected; }
private:
	bool connected;
	bool frame_flag;
	BYTE cache_idx;
	BYTE camera_id;
	WSADATA wsadata;
	SOCKET m_clientSock;
	SOCKADDR_IN m_ClientAddr;
	ADDRINFO* host_domainAddr;
	std::vector<int> encode_param = { cv::IMWRITE_JPEG_QUALITY, IMG_QUALITY };
	void set_cameraId();	
	void set_connection();
	void disconnection();
};

