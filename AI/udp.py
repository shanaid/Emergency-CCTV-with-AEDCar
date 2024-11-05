import socket
import cv2

class UdpSender:
    def __init__(self, ip, port, camera_id):
        self.server_ip = ip
        self.server_port = port
        self.camera_id = camera_id
        self.mtu = 1500
        self.udp_header_size = 28
        self.info_size = 4
        self.packet_size = self.mtu - self.udp_header_size
        self.img_seg_size = self.packet_size - self.info_size
        self.img_quality = 20
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.max_cache = 3
        self.cache_id = 0
        self.frame_flag = False

    def send_frame(self, frame):
        try:
            _, encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.img_quality])
            bytes_data = encoded_frame.tobytes()
            total_bytes_sent = 0
            num = 0
            img_packet_size = len(bytes_data)

            while total_bytes_sent < img_packet_size:
                chunk_size = min(self.img_seg_size, img_packet_size - total_bytes_sent)
                buffer = bytearray(self.packet_size + self.info_size)

                buffer[0] = 0 if self.frame_flag else 255
                buffer[1] = self.camera_id
                buffer[2] = num
                buffer[3] = self.cache_id

                buffer[4:4 + chunk_size] = bytes_data[total_bytes_sent:total_bytes_sent + chunk_size]

                sent_bytes = self.sock.sendto(buffer[:4 + chunk_size], (self.server_ip, self.server_port))

                if sent_bytes == 0:
                    print("Error sending packet.")
                    return

                total_bytes_sent += chunk_size
                num += 1

            print(f"Packet sent: {total_bytes_sent}, Seg sent: {num}")

            self.cache_id = (self.cache_id + 1) % self.max_cache
            self.frame_flag = not self.frame_flag

        except Exception as e:
            print(f"Error in send_frame: {e}")


    def close(self):
        self.sock.close()
