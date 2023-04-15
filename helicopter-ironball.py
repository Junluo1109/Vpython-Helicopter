from vpython import *

N = 71                 #介質個數(球的個數)
size = 0.4             #介質的大小(球的半徑)
size2 = 0.8            #鐵球的半徑
m = 3                  #介質的質量(球的質量)
m2 = 50                #鐵球的質量
k = 30000.0            #每一小段彈簧的彈力常數
d = 0.6                #介質之間的初始間隔
g = vector(0, -9.8, 0) #重力加速度
rho = 1.225            #空氣密度 @ 1013.25 hPa and 15°C
A = pi*(size**2)       #球體截面積
Cd = 0.47              #球體阻力係數
V_h = vector(15, 0, 0) #直升機速度

# Set up the scene
scene = canvas(title='Helicopter with Flexible Cable', center = vector(-5, -10, 0), background=vec(0.6,0.8,0.8))

helicopter = box(pos=vector(0, 0, 0), size=vector(4, 4, 4), color=color.white, texture = 'https://i.imgur.com/njd7PQx.png')

ball =[]
for i in range(N):
    if i == (N - 1):
        ball.append(sphere(radius=size2, pos=vec(0, -i*d, 0), v=vec(0,0,0), color = color.black))
    else:    
        ball.append(sphere(radius=size, pos=vec(0, -i*d, 0), v=vec(0,0,0), texture = 'https://i.imgur.com/QNuxJhD.jpg'))

spring = []
for i in range(N-1):
    spring.append(helix(radius = size/2.0, thickness = d/15.0, pos=vec(0, -i*d, 0), axis=vec(0, -d, 0)))

def SpringForce(r):    #彈力
    return - k*(mag(r)-d)*norm(r)

def DragForce(v, C): # 空氣阻力：1/2*空氣密度*球速度^2*球體阻力係數*求迎風面積
    return vector((1/2)*rho*(v.x**2)*C*A, 0, 0) 

t, dt = 0, 0.001

while True:
    rate(1000)
    t += dt

    # Update the position of the helicopter
    helicopter.pos = helicopter.pos + V_h * dt
    scene.camera.pos = scene.camera.pos + V_h * dt
    #在每次迴圈中調整彈簧位置與長度
    for i in range(N - 1):
        spring[i].pos = ball[i].pos
        spring[i].axis = ball[i + 1].pos - ball[i].pos

    #計算每一個球受相鄰兩條彈簧的彈力所造成的加速度
    for i in range(N):      

        if i == 0: # 連接直升機那顆球速度跟直升機一樣
            ball[i].v = V_h

        elif i == N-1:  # 鐵球(僅連接一顆彈簧)
            F_g = m2*g
            F_t = SpringForce(spring[i - 1].axis)
            F_drag = -DragForce(ball[i].v, Cd)
            F_total = F_g + F_t + F_drag
            ball[i].v += (F_total/m2)*dt

        else: # 中間的球
            F_g = m*g
            F_t = SpringForce(spring[i - 1].axis) + -SpringForce(spring[i].axis)
            F_drag = -DragForce(ball[i].v, Cd)
            F_total = F_g + F_t + F_drag
            ball[i].v += (F_total/m)*dt

        ball[i].pos += ball[i].v*dt

