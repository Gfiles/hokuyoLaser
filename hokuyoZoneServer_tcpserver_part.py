import socket
import threading

# TCP Server class to handle client connections and message sending
class TCPServer:
	def __init__(self, host='0.0.0.0', port=65432):
		self.host = host
		self.port = port
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.server_socket.bind((self.host, self.port))
		self.server_socket.listen()
		self.clients = []
		self.running = True
		self.lock = threading.Lock()

	def start(self):
		thread = threading.Thread(target=self.accept_clients)
		thread.daemon = True
		thread.start()

	def accept_clients(self):
		print(f"TCP Server listening on {self.host}:{self.port}")
		while self.running:
			try:
				client_socket, addr = self.server_socket.accept()
				print(f"Client connected from {addr}")
				with self.lock:
					self.clients.append(client_socket)
				threading.Thread(target=self.handle_client, args=(client_socket, addr), daemon=True).start()
			except Exception as e:
				print(f"Error accepting clients: {e}")

	def handle_client(self, client_socket, addr):
		try:
			while self.running:
				data = client_socket.recv(1024)
				if not data:
					break
			print(f"Client disconnected from {addr}")
		except Exception as e:
			print(f"Error handling client {addr}: {e}")
		finally:
			with self.lock:
				if client_socket in self.clients:
					self.clients.remove(client_socket)
			client_socket.close()

	def send(self, message):
		with self.lock:
			for client in self.clients[:]:
				try:
					client.sendall(message)
				except Exception as e:
					print(f"Error sending to client: {e}")
					self.clients.remove(client)
					client.close()

	def stop(self):
		self.running = False
		self.server_socket.close()
		with self.lock:
			for client in self.clients:
				client.close()
			self.clients.clear()
