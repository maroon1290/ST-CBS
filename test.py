import math

spin = input("회전 방향: ")
theta = input("각도 입력: ") # -3.14 ~ 3.14

# theta to degree
theta = float(theta) * 180 / math.pi

if spin == "cw":
    if theta < 0:
        theta = -theta
    else:
        theta = 360 - theta

elif spin == "ccw":
    if theta < 0:
        theta = 360 + theta
    else:
        theta = theta

print("각도: ", theta)