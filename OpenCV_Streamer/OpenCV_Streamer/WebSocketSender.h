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

//MTU�� ���� 1500���� �����Ǿ� �ִ�.
//CMD���� �̸� ������ �� ������ ������� �ʴ´ٰ� �Ѵ�.
//���� ��Ŷ�� 1500�� ���߾� �߶� ������ ������ �ʿ��ϴ�.
#define PORT			5432
#define MTU				1500
#define UDP_HEADER_SIZE 28
#define INFO_SIZE		4						//ī�޶� ��ȣ �� �̹��� ��ȣ�� ��� ������ ����� ���� ��Ŷ ���(UDP ����� ����)�� ũ��(����Ʈ) - End_Flag, CameraID, ImageSegNum
#define PACKET_SIZE		MTU - UDP_HEADER_SIZE	//MTU - UPD-Header = 1500 - 28 = 1472
#define IMG_SEG_SIZE	PACKET_SIZE - INFO_SIZE
#define IMG_QUALITY		20						//jpeg �������� ��ȯ�Ҷ� ȭ�� ����
//#define SERVER_IP		"127.0.0.1"
#define SERVER_IP		"43.202.61.242"
#define SERVER_DOMAIN	"j11b209.p.ssafy.io"

#define MAX_CACHE 3

//�̹��� Mat�� ũ��
constexpr int IMG_FULL_SIZE = 640 * 480 * 3;


/// <summary>
/// UDP�� ����Ͽ� OpenCV�� �̹����� �����ϴ� Ŭ����
/// 
/// �Ϲ����� ��ǻ���� MTU�� 1500 ����Ʈ�Դϴ�. �� MTU�� ������ �� ������ �ϴ��� �������� �ʾҴٰ� �����ϰڽ��ϴ�.
/// UDP�� ��Ŷ�� ������ �� 1500��ŭ ������ ���������� UDP ����� ũ�� 28����Ʈ�� ���� 1472����Ʈ�� ������ �����մϴ�.
/// ���� �� UDP ��Ŷ ���ۿ��� ���� �� �ִ� �����ʹ� 1472����Ʈ �Ǵ� MTU - UDP HEADER �Դϴ�.
/// ������ ī�޶� �������� ��찡 �����Ƿ� �� ��Ŷ���� ī�޶� ������ �ʿ��մϴ�. ���� �̹��� ���׸�Ʈ ���� ���� �ʿ��մϴ�.
/// �׷��Ƿ� �̷��� ����(ī�޶� ��ȣ, ���׸�Ʈ ��ȣ, ������ ���׸�Ʈ ����)�� ���� 3 ����Ʈ�� �Բ� �����ؾ��մϴ�.
/// �׷��Ƿ� ������ ���۰����� �̹��� �����ʹ� �ѹ��� MTU - UDP_HEADER - INFO_HEADER�� 1469 ����Ʈ�Դϴ�.
/// 
/// MAT ������ OPENCV�� �⺻���� �̹��� �����Դϴ�. ũ��� �̹��� ���� * �ʺ� * 3 (RGB)�� �ſ� ū �����̸� �̸� �״�� �����ϴ� ���� ��ȿ�����Դϴ�.
/// ����, �̷��� MAT ���¸� �״�� ����Ϸ��� ����Ʈ���忡�� OPENCV�� ��ġ�Ǿ� �־���մϴ�.
/// ���� OPENCV���� �⺻ �����ϴ� JPEG �������� ��ȯ�ϴ� �޼��带 ����Ͽ� �̸� ������ ��ȯ���� �ٷ� ����� �� �ֵ��� �մϴ�.
/// ������̳� �̹��� �������� ���� ũ�Ⱑ �ٸ����� ���� 60000 ~ 70000 ����Ʈ ������ ũ�⸦ �����ϴ�.
/// �̸� �ռ� ����� ���۰����� �̹��� ��Ŷ�� ũ�� 1469 ����Ʈ�� ������ �����մϴ�. sendframe_via_udp ����
/// 
/// ���������� ���������� �󸶸�ŭ�� ��Ŷ�� �������� �� �� ���� ������ ������ ���׸�Ʈ ������ ���� ���� ���� �߰��� ��ü ��Ŷ�� ũ�⸦ �� �� �ֽ��ϴ�.
/// ������ ���׸�Ʈ�� ��� ������� ���� ��Ŷ�� ����Ʈ����� �����մϴ�.
/// 
/// </summary>
class WebSocketSender
{
public:
	WebSocketSender();
	~WebSocketSender();

	//������ ����
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

