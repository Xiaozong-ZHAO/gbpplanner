// #include "raylib.h"
// #include "raymath.h"
// #include "box2d/box2d.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

// // 常量定义
// #define SCREEN_WIDTH 1200
// #define SCREEN_HEIGHT 800
// #define MAX_ROBOTS 8
// #define ROBOT_RADIUS 15.0f
// #define PAYLOAD_RADIUS 40.0f
// #define MAX_TRAIL_POINTS 100

// // Box2D常量
// #define PIXELS_PER_METER 50.0f
// #define METERS_PER_PIXEL (1.0f / PIXELS_PER_METER)

// // 将Box2D坐标转换为屏幕坐标
// Vector2 Box2DToScreen(b2Vec2 pos) {
//     Vector2 screenPos;
//     screenPos.x = pos.x * PIXELS_PER_METER;
//     screenPos.y = pos.y * PIXELS_PER_METER;
//     return screenPos;
// }

// // 将屏幕坐标转换为Box2D坐标
// b2Vec2 ScreenToBox2D(Vector2 pos) {
//     b2Vec2 box2dPos;
//     box2dPos.x = pos.x * METERS_PER_PIXEL;
//     box2dPos.y = pos.y * METERS_PER_PIXEL;
//     return box2dPos;
// }

// // 机器人结构
// typedef struct {
//     b2Body* body;
//     Color color;
//     bool selected;
//     Vector2 pushDirection;  // 推动方向
//     float pushForce;        // 推动力大小
// } Robot;

// // 载荷结构
// typedef struct {
//     b2Body* body;
//     Vector2 trailPoints[MAX_TRAIL_POINTS];
//     int trailCount;
//     int trailUpdateCounter;
// } Payload;

// // 创建机器人
// Robot CreateRobot(b2World* world, Vector2 position, Color robotColor) {
//     Robot robot;
    
//     // 创建机器人的物理属性
//     b2BodyDef bodyDef;
//     bodyDef.type = b2_dynamicBody;
//     b2Vec2 box2dPos = ScreenToBox2D(position);
//     bodyDef.position = box2dPos;
//     bodyDef.linearDamping = 0.5f;  // 添加阻尼减少振荡
//     robot.body = world->CreateBody(&bodyDef);
    
//     // 创建机器人的圆形碰撞形状
//     b2CircleShape circleShape;
//     circleShape.m_radius = ROBOT_RADIUS * METERS_PER_PIXEL;
    
//     // 设置物理材质属性
//     b2FixtureDef fixtureDef;
//     fixtureDef.shape = &circleShape;
//     fixtureDef.density = 1.0f;
//     fixtureDef.friction = 0.8f;     // 增加摩擦力以便更好地推动物体
//     fixtureDef.restitution = 0.1f;  // 弹性系数
//     robot.body->CreateFixture(&fixtureDef);
    
//     robot.color = robotColor;
//     robot.selected = false;
//     robot.pushDirection = (Vector2){0, 0};
//     robot.pushForce = 10.0f;  // 默认推力
    
//     return robot;
// }

// // 获取机器人位置
// Vector2 GetRobotPosition(b2World* world, Robot robot) {
//     b2Vec2 pos = robot.body->GetPosition();
//     return Box2DToScreen(pos);
// }

// // 设置机器人位置
// void SetRobotPosition(b2World* world, Robot* robot, Vector2 newPos) {
//     b2Vec2 box2dPos = ScreenToBox2D(newPos);
//     float angle = robot->body->GetAngle();
//     robot->body->SetTransform(box2dPos, angle);
//     robot->body->SetAwake(true);
// }

// // 绘制机器人
// void DrawRobot(b2World* world, Robot robot, int id) {
//     Vector2 pos = GetRobotPosition(world, robot);
    
//     // 绘制机器人ID
//     char robotId[3];
//     sprintf(robotId, "%d", id + 1);
//     DrawText(robotId, pos.x - 5, pos.y - 10, 20, DARKGRAY);
    
//     // 绘制机器人
//     DrawCircleV(pos, ROBOT_RADIUS, robot.selected ? WHITE : robot.color);
//     DrawCircleLines(pos.x, pos.y, ROBOT_RADIUS, robot.selected ? robot.color : Fade(BLACK, 0.3f));
    
//     // 如果有推力方向，绘制一个箭头表示
//     if (Vector2Length(robot.pushDirection) > 0.1f) {
//         Vector2 pushEnd = Vector2Add(pos, Vector2Scale(robot.pushDirection, ROBOT_RADIUS * 1.5f));
//         DrawLineV(pos, pushEnd, RED);
        
//         // 绘制箭头尖
//         float angle = atan2f(robot.pushDirection.y, robot.pushDirection.x);
//         Vector2 arrowLeft = {
//             pushEnd.x - 8 * cosf(angle + PI/6),
//             pushEnd.y - 8 * sinf(angle + PI/6)
//         };
//         Vector2 arrowRight = {
//             pushEnd.x - 8 * cosf(angle - PI/6),
//             pushEnd.y - 8 * sinf(angle - PI/6)
//         };
//         DrawLineV(pushEnd, arrowLeft, RED);
//         DrawLineV(pushEnd, arrowRight, RED);
//     }
// }

// // 创建载荷
// Payload CreatePayload(b2World* world, Vector2 position) {
//     Payload payload;
    
//     // 创建载荷的物理属性
//     b2BodyDef bodyDef;
//     bodyDef.type = b2_dynamicBody;
//     b2Vec2 box2dPos = ScreenToBox2D(position);
//     bodyDef.position = box2dPos;
//     bodyDef.linearDamping = 0.2f;  // 添加阻尼减少振荡
//     payload.body = world->CreateBody(&bodyDef);
    
//     // 创建载荷的圆形碰撞形状
//     b2CircleShape circleShape;
//     circleShape.m_radius = PAYLOAD_RADIUS * METERS_PER_PIXEL;
    
//     // 设置物理材质属性
//     b2FixtureDef fixtureDef;
//     fixtureDef.shape = &circleShape;
//     fixtureDef.density = 5.0f;   // 载荷比机器人重
//     fixtureDef.friction = 0.5f;  // 摩擦力适中
//     fixtureDef.restitution = 0.1f;  // 弹性系数
//     payload.body->CreateFixture(&fixtureDef);
    
//     payload.trailCount = 0;
//     payload.trailUpdateCounter = 0;
    
//     return payload;
// }

// // 获取载荷位置
// Vector2 GetPayloadPosition(b2World* world, Payload payload) {
//     b2Vec2 pos = payload.body->GetPosition();
//     return Box2DToScreen(pos);
// }

// // 更新载荷轨迹
// void UpdatePayloadTrail(b2World* world, Payload* payload) {
//     payload->trailUpdateCounter++;
//     if (payload->trailUpdateCounter >= 5) {  // 每5帧更新一次轨迹
//         payload->trailUpdateCounter = 0;
        
//         Vector2 pos = GetPayloadPosition(world, *payload);
        
//         // 添加新轨迹点或移动旧轨迹点
//         if (payload->trailCount < MAX_TRAIL_POINTS) {
//             payload->trailPoints[payload->trailCount] = pos;
//             payload->trailCount++;
//         } else {
//             // 移动所有轨迹点
//             for (int i = 0; i < MAX_TRAIL_POINTS - 1; i++) {
//                 payload->trailPoints[i] = payload->trailPoints[i + 1];
//             }
//             payload->trailPoints[MAX_TRAIL_POINTS - 1] = pos;
//         }
//     }
// }

// // 清除载荷轨迹
// void ClearPayloadTrail(Payload* payload) {
//     payload->trailCount = 0;
// }

// // 绘制载荷
// void DrawPayload(b2World* world, Payload payload) {
//     Vector2 pos = GetPayloadPosition(world, payload);
    
//     // 绘制轨迹
//     if (payload.trailCount > 1) {
//         for (int i = 0; i < payload.trailCount - 1; i++) {
//             float alpha = (float)i / payload.trailCount;
//             DrawLineV(payload.trailPoints[i], payload.trailPoints[i + 1], Fade(GRAY, alpha));
//         }
//     }
    
//     // 绘制载荷
//     DrawCircleV(pos, PAYLOAD_RADIUS, DARKGRAY);
    
//     // 显示速度信息
//     b2Vec2 velocity = payload.body->GetLinearVelocity();
//     char velocityText[50];
//     sprintf(velocityText, "V: %.2f, %.2f", velocity.x, velocity.y);
//     DrawText(velocityText, pos.x - 40, pos.y - PAYLOAD_RADIUS - 20, 15, DARKGRAY);
// }

// // 计算机器人的最佳推动位置和方向
// void CalculatePushStrategy(Robot* robots, int robotCount, Payload payload, Vector2 targetPosition, b2World* world) {
//     Vector2 payloadPos = GetPayloadPosition(world, payload);
    
//     // 计算载荷到目标的方向
//     Vector2 toTarget = Vector2Subtract(targetPosition, payloadPos);
//     float distToTarget = Vector2Length(toTarget);
    
//     // 如果已经非常接近目标，则不需要推动
//     if (distToTarget < 5.0f) {
//         for (int i = 0; i < robotCount; i++) {
//             robots[i].pushDirection = (Vector2){0, 0}; // 清除推力
//         }
//         return;
//     }
    
//     // 标准化方向向量
//     Vector2 targetDir = Vector2Normalize(toTarget);
    
//     // 根据机器人数量均匀分配推动任务
//     for (int i = 0; i < robotCount; i++) {
//         // 计算每个机器人相对于载荷的最佳位置角度
//         float angle = (2.0f * PI * i) / robotCount;
        
//         // 对角度进行调整，使得有机器人从推动方向推动
//         float angleOffset = PI/4; // 调整这个值可以改变推动策略
//         float pushAngle = atan2f(targetDir.y, targetDir.x) + angle * angleOffset - PI/2;
        
//         // 计算推动方向，始终朝向载荷中心的方向
//         Vector2 robotPos = GetRobotPosition(world, robots[i]);
//         Vector2 toPayload = Vector2Subtract(payloadPos, robotPos);
//         float distToPayload = Vector2Length(toPayload);
        
//         // 如果机器人离载荷很近，则计算切线方向推动
//         if (distToPayload < PAYLOAD_RADIUS + ROBOT_RADIUS * 3.0f) {
//             // 计算切线方向，使得机器人能更有效地推动载荷
//             Vector2 normalDir = Vector2Normalize(toPayload);
            
//             // 使用目标方向的投影来决定推动方向
//             float dotProduct = normalDir.x * targetDir.x + normalDir.y * targetDir.y;
            
//             // 如果机器人在载荷运动方向的后半部分，则直接推动
//             if (dotProduct < 0) {
//                 robots[i].pushDirection = targetDir;
//             } else {
//                 // 计算切线方向
//                 Vector2 tangentDir = {-normalDir.y, normalDir.x};
//                 float tangentDot = tangentDir.x * targetDir.x + tangentDir.y * targetDir.y;
                
//                 // 选择与目标方向夹角较小的切线方向
//                 if (tangentDot < 0) {
//                     tangentDir = (Vector2){-tangentDir.x, -tangentDir.y};
//                 }
                
//                 // 混合切线和径向方向
//                 robots[i].pushDirection = Vector2Normalize(
//                     Vector2Add(
//                         Vector2Scale(tangentDir, 0.7f),
//                         Vector2Scale(targetDir, 0.3f)
//                     )
//                 );
//             }
            
//             // 设置推力大小 - 距离目标越远推力越大
//             robots[i].pushForce = 10.0f + 20.0f * (distToTarget / SCREEN_WIDTH);
//         } else {
//             // 如果机器人离载荷很远，需要先靠近载荷
//             robots[i].pushDirection = Vector2Normalize(toPayload);
//             robots[i].pushForce = 15.0f; // 移动到载荷的力
//         }
//     }
// }

// // 应用推力
// void ApplyPushForces(Robot* robots, int robotCount, b2World* world) {
//     for (int i = 0; i < robotCount; i++) {
//         if (!robots[i].selected && Vector2Length(robots[i].pushDirection) > 0.1f) {
//             b2Vec2 force(
//                 robots[i].pushDirection.x * robots[i].pushForce,
//                 robots[i].pushDirection.y * robots[i].pushForce
//             );
//             robots[i].body->ApplyForceToCenter(force, true);
//         }
//     }
// }

// int main(void) {
//     // 初始化窗口
//     InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "机器人推动物体模拟");
//     SetTargetFPS(60);
    
//     // 创建Box2D世界
//     b2Vec2 gravity(0.0f, 0.0f);  // 无重力，纯粹的推动效果
//     b2World world(gravity);
    
//     // 创建边界墙壁
//     b2BodyDef groundBodyDef;
//     groundBodyDef.type = b2_staticBody;
//     b2Body* groundBody = world.CreateBody(&groundBodyDef);
    
//     // 下边界
//     {
//         b2EdgeShape edgeShape;
//         edgeShape.SetTwoSided(b2Vec2(0, SCREEN_HEIGHT * METERS_PER_PIXEL), 
//                               b2Vec2(SCREEN_WIDTH * METERS_PER_PIXEL, SCREEN_HEIGHT * METERS_PER_PIXEL));
        
//         b2FixtureDef edgeFixtureDef;
//         edgeFixtureDef.shape = &edgeShape;
//         edgeFixtureDef.friction = 0.3f;
//         groundBody->CreateFixture(&edgeFixtureDef);
//     }
    
//     // 上边界
//     {
//         b2EdgeShape edgeShape;
//         edgeShape.SetTwoSided(b2Vec2(0, 0), 
//                               b2Vec2(SCREEN_WIDTH * METERS_PER_PIXEL, 0));
        
//         b2FixtureDef edgeFixtureDef;
//         edgeFixtureDef.shape = &edgeShape;
//         edgeFixtureDef.friction = 0.3f;
//         groundBody->CreateFixture(&edgeFixtureDef);
//     }
    
//     // 左边界
//     {
//         b2EdgeShape edgeShape;
//         edgeShape.SetTwoSided(b2Vec2(0, 0), 
//                               b2Vec2(0, SCREEN_HEIGHT * METERS_PER_PIXEL));
        
//         b2FixtureDef edgeFixtureDef;
//         edgeFixtureDef.shape = &edgeShape;
//         edgeFixtureDef.friction = 0.3f;
//         groundBody->CreateFixture(&edgeFixtureDef);
//     }
    
//     // 右边界
//     {
//         b2EdgeShape edgeShape;
//         edgeShape.SetTwoSided(b2Vec2(SCREEN_WIDTH * METERS_PER_PIXEL, 0), 
//                               b2Vec2(SCREEN_WIDTH * METERS_PER_PIXEL, SCREEN_HEIGHT * METERS_PER_PIXEL));
        
//         b2FixtureDef edgeFixtureDef;
//         edgeFixtureDef.shape = &edgeShape;
//         edgeFixtureDef.friction = 0.3f;
//         groundBody->CreateFixture(&edgeFixtureDef);
//     }
    
//     // 创建载荷
//     Payload payload = CreatePayload(&world, (Vector2){SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f});
    
//     // 创建机器人数组
//     Robot robots[MAX_ROBOTS];
//     int robotCount = 4;
//     Color robotColors[6] = {RED, BLUE, GREEN, YELLOW, PURPLE, ORANGE};
//     float robotDistance = PAYLOAD_RADIUS * 2.5f;
    
//     // 初始化机器人
//     for (int i = 0; i < robotCount; i++) {
//         float angle = (2.0f * PI * i) / robotCount;
//         Vector2 position = {
//             SCREEN_WIDTH / 2.0f + robotDistance * cos(angle),
//             SCREEN_HEIGHT / 2.0f + robotDistance * sin(angle)
//         };
        
//         robots[i] = CreateRobot(&world, position, robotColors[i % 6]);
//     }
    
//     // 目标位置
//     Vector2 targetPosition = {SCREEN_WIDTH * 0.8f, SCREEN_HEIGHT * 0.7f};
//     bool targetMoving = false;
    
//     // 选中的机器人
//     int selectedRobot = -1;
    
//     // 鼠标拖动
//     b2Body* mouseBody = nullptr;
//     b2MouseJoint* mouseJoint = nullptr;
    
//     // 创建一个静态体用于鼠标关节
//     b2BodyDef mouseBodyDef;
//     mouseBodyDef.type = b2_staticBody;
//     mouseBody = world.CreateBody(&mouseBodyDef);
    
//     // 主游戏循环
//     while (!WindowShouldClose()) {
//         // 处理用户输入
//         if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
//             Vector2 mousePos = GetMousePosition();
            
//             // 检查是否点击了目标点
//             if (CheckCollisionPointCircle(mousePos, targetPosition, 15)) {
//                 targetMoving = true;
//             } else {
//                 // 检查是否选中了机器人
//                 for (int i = 0; i < robotCount; i++) {
//                     Vector2 robotPos = GetRobotPosition(&world, robots[i]);
//                     if (CheckCollisionPointCircle(mousePos, robotPos, ROBOT_RADIUS)) {
//                         selectedRobot = i;
//                         robots[i].selected = true;
                        
//                         // 创建鼠标关节
//                         b2MouseJointDef jointDef;
//                         jointDef.bodyA = mouseBody;
//                         jointDef.bodyB = robots[i].body;
//                         jointDef.target = ScreenToBox2D(mousePos);
//                         jointDef.maxForce = 1000.0f * robots[i].body->GetMass();
//                         jointDef.stiffness = 100.0f;
//                         jointDef.damping = 0.7f;
                        
//                         mouseJoint = (b2MouseJoint*)world.CreateJoint(&jointDef);
//                         break;
//                     }
//                 }
//             }
//         }
        
//         // 更新鼠标拖动
//         if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
//             Vector2 mousePos = GetMousePosition();
            
//             if (targetMoving) {
//                 targetPosition = mousePos;
//             } else if (mouseJoint != nullptr && selectedRobot != -1) {
//                 b2Vec2 target = ScreenToBox2D(mousePos);
//                 mouseJoint->SetTarget(target);
//             }
//         }
        
//         // 释放鼠标
//         if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
//             targetMoving = false;
            
//             if (mouseJoint != nullptr) {
//                 world.DestroyJoint(mouseJoint);
//                 mouseJoint = nullptr;
                
//                 if (selectedRobot != -1) {
//                     robots[selectedRobot].selected = false;
//                     selectedRobot = -1;
//                 }
//             }
//         }
        
//         // 添加或删除机器人
//         if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
//             Vector2 mousePos = GetMousePosition();
            
//             // 检查是否点击了现有机器人（删除）
//             bool robotClicked = false;
//             for (int i = 0; i < robotCount; i++) {
//                 Vector2 robotPos = GetRobotPosition(&world, robots[i]);
//                 if (CheckCollisionPointCircle(mousePos, robotPos, ROBOT_RADIUS)) {
//                     // 删除机器人
//                     if (robotCount > 2) {  // 保持至少2个机器人
//                         // 删除身体
//                         world.DestroyBody(robots[i].body);
                        
//                         // 移动数组中的元素
//                         for (int j = i; j < robotCount - 1; j++) {
//                             robots[j] = robots[j + 1];
//                         }
                        
//                         robotCount--;
//                         robotClicked = true;
//                     }
//                     break;
//                 }
//             }
            
//             // 如果没有点击现有机器人，添加新机器人
//             if (!robotClicked && robotCount < MAX_ROBOTS) {
//                 robots[robotCount] = CreateRobot(&world, mousePos, robotColors[robotCount % 6]);
//                 robotCount++;
//             }
//         }
        
//         // R键重置模拟
//         if (IsKeyPressed(KEY_R)) {
//             // 重置载荷位置
//             b2Vec2 centerPos(SCREEN_WIDTH / 2.0f * METERS_PER_PIXEL, SCREEN_HEIGHT / 2.0f * METERS_PER_PIXEL);
//             float angle = payload.body->GetAngle();
//             payload.body->SetTransform(centerPos, angle);
//             payload.body->SetLinearVelocity(b2Vec2(0, 0));
//             payload.body->SetAngularVelocity(0);
            
//             // 清除轨迹
//             ClearPayloadTrail(&payload);
            
//             // 删除所有现有机器人
//             for (int i = 0; i < robotCount; i++) {
//                 world.DestroyBody(robots[i].body);
//             }
            
//             // 重新创建4个机器人
//             robotCount = 4;
//             for (int i = 0; i < robotCount; i++) {
//                 float angle = (2.0f * PI * i) / robotCount;
//                 Vector2 position = {
//                     SCREEN_WIDTH / 2.0f + robotDistance * cos(angle),
//                     SCREEN_HEIGHT / 2.0f + robotDistance * sin(angle)
//                 };
                
//                 robots[i] = CreateRobot(&world, position, robotColors[i % 6]);
//             }
//         }
        
//         // 箭头键为载荷添加外力（模拟外部干扰）
//         b2Vec2 externalForce(0, 0);
//         if (IsKeyDown(KEY_RIGHT)) externalForce.x += 10.0f;
//         if (IsKeyDown(KEY_LEFT)) externalForce.x -= 10.0f;
//         if (IsKeyDown(KEY_DOWN)) externalForce.y += 10.0f;
//         if (IsKeyDown(KEY_UP)) externalForce.y -= 10.0f;
        
//         if (externalForce.x != 0 || externalForce.y != 0) {
//             payload.body->ApplyForceToCenter(externalForce, true);
//         }
        
//         // 计算机器人推动策略并应用推力
//         CalculatePushStrategy(robots, robotCount, payload, targetPosition, &world);
//         ApplyPushForces(robots, robotCount, &world);
        
//         // 更新Box2D世界
//         float timeStep = 1.0f / 60.0f;
//         int32 velocityIterations = 6;
//         int32 positionIterations = 2;
//         world.Step(timeStep, velocityIterations, positionIterations);
        
//         // 更新载荷轨迹
//         UpdatePayloadTrail(&world, &payload);
        
//         // 绘制
//         BeginDrawing();
//         ClearBackground(RAYWHITE);
        
//         // 绘制目标位置
//         DrawCircleV(targetPosition, 10, Fade(GREEN, 0.3f));
//         DrawCircleLines(targetPosition.x, targetPosition.y, 15, GREEN);
//         DrawText("Target", targetPosition.x + 20, targetPosition.y, 20, DARKGREEN);
        
//         // 绘制从载荷到目标的方向线
//         Vector2 payloadPos = GetPayloadPosition(&world, payload);
//         DrawLineV(payloadPos, targetPosition, Fade(GREEN, 0.3f));
        
//         // 绘制载荷和轨迹
//         DrawPayload(&world, payload);
        
//         // 绘制机器人
//         for (int i = 0; i < robotCount; i++) {
//             DrawRobot(&world, robots[i], i);
//         }
        
//         // 绘制帮助文本
//         DrawRectangle(0, 0, SCREEN_WIDTH, 40, Fade(SKYBLUE, 0.5f));
//         DrawText("Left-click to drag: robots or target | Right-click to add/remove robots | R to reset | Arrow keys to apply external force", 
//                  10, 10, 20, DARKBLUE);
        
//         // 绘制状态信息
//         char statusText[128];
//         Vector2 distance = Vector2Subtract(targetPosition, payloadPos);
//         sprintf(statusText, "Robot number: %d | Distance to target: %.2f", 
//                 robotCount, Vector2Length(distance));
//         DrawText(statusText, 10, SCREEN_HEIGHT - 30, 20, DARKGRAY);
        
//         EndDrawing();
//     }
    
//     // 如果有鼠标关节，释放它
//     if (mouseJoint != nullptr) {
//         world.DestroyJoint(mouseJoint);
//     }
    
//     // 销毁所有物体
//     for (int i = 0; i < robotCount; i++) {
//         world.DestroyBody(robots[i].body);
//     }
    
//     world.DestroyBody(payload.body);
//     world.DestroyBody(groundBody);
//     world.DestroyBody(mouseBody);
    
//     // 关闭窗口
//     CloseWindow();
    
//     return 0;
// }


/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)

// Define all parameters in the appropriate config file (default: config/config.json)
/**************************************************************************************/
#define RLIGHTS_IMPLEMENTATION // needed to be defined once for the lights shader
#include <iostream>
#include <Utils.h>

#include <DArgs.h>

#include <Globals.h>
#include <Simulator.h>

Globals globals;
int main(int argc, char *argv[]){
    
    srand((int)globals.SEED);                                   // Initialise random seed   
    DArgs::DArgs dargs(argc, argv);                             // Parse config file argument --cfg <file.json>
    if (globals.parse_global_args(dargs)) return EXIT_FAILURE;  
    
    Simulator* sim = new Simulator();       // Initialise the simulator
    globals.RUN = true;
    while (globals.RUN){

        // sim->eventHandler();                // Capture keypresses or mouse events             
        // sim->createOrDeleteRobots();        
        // sim->timestep();
        sim->createSingleRobot();
        
        sim->draw();
        // sim->draw_payloads();
    }

    delete sim;

    return 0;
}    
