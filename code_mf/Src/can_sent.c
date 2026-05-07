

#include "can_sent.h"

// void can_sent()
//{
 //   while (1)
  //  {
        // if (rc_receive_state == RC_OFFLINE || yaw_6020_state == GM6020_DIE || pitch_6020_state == GM6020_DIE)//遥控器离线，电机过温保护；
        // {
        //     can_rm_cmd_all(0, 0, 0, 0, 0, 0, 0, 0);
        //     can_xiaomi_cmd_all(CLOSE_XIAOMI);
        // }
        // else
        // {
//              switch (RC_attation )
//              {
//              case ( 0 )://全车关电
//                  {
//                      can_rm_cmd_all(0, 0, 0, 0, 0, 0, 0, 0);
//                      can_xiaomi_cmd_all(CLOSE_XIAOMI);
//                      break;
//                  }
//
//              case ( 1 )://运动模式|小陀螺模式
//                 {
//                     can_rm_cmd_all
//                    (1000,
//                    1000,
//                    1000,
//                    1000,
//                    6000,
//                    1000,
//                    1000,
//                    1000);
//
//
//               //    can_xiaomi_cmd_all(OPEN_XIAOMI);
//              //        break;
//              //    }
//              //
//              // default:
//              //     {
//              //         can_rm_cmd_all(0, 0, 0, 0, 0, 0, 0, 0);
//              //         can_xiaomi_cmd_all(CLOSE_XIAOMI);
//              //         break;
//              //     }
//              // }
//
//             osDelay(1);
//
//     }
// }

//         if (rc_receive_state == RC_OFFLINE | yaw_6020_state == GM6020_DIE | pitch_6020_state == GM6020_DIE)
//         //????????????????
//         {
//             can_xiaomi_cmd_all(CLOSE_XIAOMI);
//         }
//         else //?????????????????п??????
//         {
//             if (rc_s0 == 2)//             {

//                 can_xiaomi_cmd_all(CLOSE_XIAOMI);
//             }
//             else if (rc_s0 == 3 | rc_s0 == 1) //?????
//             {
//                 can_xiaomi_cmd_all(OPEN_XIAOMI);
//             }
//
//             else //??????????????л??????????
//             {
//                 can_xiaomi_cmd_all(CLOSE_XIAOMI);
//             }
//         }
//
//         osDelay(1);
//     }

//
//
// void can_rm_cmd_all(int16_t chassis_id1, int16_t chassis_id2,
//                     int16_t chassis_id3, int16_t chassis_id4,
//                     int16_t yaw_id5,     int16_t track_id6,
//                     int16_t track_id7,   int16_t shoot_id8)
// {
//     CAN1_cmd_chassis(chassis_id1, chassis_id2, chassis_id3, chassis_id4);
//     CAN1_cmd_yaw(yaw_id5, 0, 0, shoot_id8);
//     CAN2_cmd_pitch(0,track_id6,track_id7,0);
// }
//
//



