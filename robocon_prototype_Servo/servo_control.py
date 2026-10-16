
import time
import serial
import pygame

# ===== SETTINGS =====
COM_PORT = "COM5"    # Change to your ESP32 port
BAUD_RATE = 115200

MIN_ANGLE = 0
MAX_ANGLE = 360

STEP = 5
MOVE_INTERVAL = 0.1  # Seconds between movements

current_angle = 0

# ===== CONNECT ESP32 =====
try:
    esp32 = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    print("Connected to ESP32 on", COM_PORT)
except serial.SerialException as e:
    print("Connection failed:", e)
    raise SystemExit

# ===== KEYBOARD WINDOW =====
pygame.init()

screen = pygame.display.set_mode((460, 230))
pygame.display.set_caption("Studica 75002 Servo Controller")

font = pygame.font.SysFont(None, 55)
small_font = pygame.font.SysFont(None, 27)

clock = pygame.time.Clock()

def send_angle(angle):
    global current_angle

    current_angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))

    command = f"A:{current_angle}\n"
    esp32.write(command.encode())

    print(f"Commanded angle: {current_angle} degrees")

send_angle(current_angle)

running = True
last_move = 0

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_SPACE:
                    send_angle(0)

        keys = pygame.key.get_pressed()
        now = time.monotonic()

        if now - last_move >= MOVE_INTERVAL:

            if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
                if current_angle < MAX_ANGLE:
                    send_angle(current_angle + STEP)
                last_move = now

            elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
                if current_angle > MIN_ANGLE:
                    send_angle(current_angle - STEP)
                last_move = now

        # ===== DISPLAY =====
        screen.fill((25, 30, 40))

        title = small_font.render(
            "Studica 75002 - ESP32 Control",
            True, (220, 220, 220)
        )

        angle_text = font.render(
            f"{current_angle} degrees",
            True, (0, 220, 150)
        )

        help_text = small_font.render(
            "UP / DOWN: Move | SPACE: Zero | ESC: Exit",
            True, (200, 200, 200)
        )

        screen.blit(title, (90, 30))
        screen.blit(angle_text, (125, 90))
        screen.blit(help_text, (25, 180))

        pygame.display.flip()
        clock.tick(60)

finally:
    esp32.close()
    pygame.quit()
    print("Controller disconnected")
