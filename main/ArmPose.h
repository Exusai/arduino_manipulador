#ifndef _ROS_unity_msgs_ArmPose_h
#define _ROS_unity_msgs_ArmPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unity_msgs
{

  class ArmPose : public ros::Msg
  {
    public:
      typedef float _q1_type;
      _q1_type q1;
      typedef float _q2_type;
      _q2_type q2;
      typedef float _q3_type;
      _q3_type q3;
      typedef float _d4_type;
      _d4_type d4;
      typedef float _q5_type;
      _q5_type q5;
      typedef float _succ_type;
      _succ_type succ;

    ArmPose():
      q1(0),
      q2(0),
      q3(0),
      d4(0),
      q5(0),
      succ(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_q1;
      u_q1.real = this->q1;
      *(outbuffer + offset + 0) = (u_q1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q1);
      union {
        float real;
        uint32_t base;
      } u_q2;
      u_q2.real = this->q2;
      *(outbuffer + offset + 0) = (u_q2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q2);
      union {
        float real;
        uint32_t base;
      } u_q3;
      u_q3.real = this->q3;
      *(outbuffer + offset + 0) = (u_q3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q3);
      union {
        float real;
        uint32_t base;
      } u_d4;
      u_d4.real = this->d4;
      *(outbuffer + offset + 0) = (u_d4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d4);
      union {
        float real;
        uint32_t base;
      } u_q5;
      u_q5.real = this->q5;
      *(outbuffer + offset + 0) = (u_q5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q5);
      union {
        float real;
        uint32_t base;
      } u_succ;
      u_succ.real = this->succ;
      *(outbuffer + offset + 0) = (u_succ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_succ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_succ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_succ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->succ);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_q1;
      u_q1.base = 0;
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q1 = u_q1.real;
      offset += sizeof(this->q1);
      union {
        float real;
        uint32_t base;
      } u_q2;
      u_q2.base = 0;
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q2 = u_q2.real;
      offset += sizeof(this->q2);
      union {
        float real;
        uint32_t base;
      } u_q3;
      u_q3.base = 0;
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q3 = u_q3.real;
      offset += sizeof(this->q3);
      union {
        float real;
        uint32_t base;
      } u_d4;
      u_d4.base = 0;
      u_d4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->d4 = u_d4.real;
      offset += sizeof(this->d4);
      union {
        float real;
        uint32_t base;
      } u_q5;
      u_q5.base = 0;
      u_q5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_q5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_q5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_q5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->q5 = u_q5.real;
      offset += sizeof(this->q5);
      union {
        float real;
        uint32_t base;
      } u_succ;
      u_succ.base = 0;
      u_succ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_succ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_succ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_succ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->succ = u_succ.real;
      offset += sizeof(this->succ);
     return offset;
    }

    virtual const char * getType() override { return "unity_msgs/ArmPose"; };
    virtual const char * getMD5() override { return "ffea4f3e3644005bdf0b21e3a7766cb5"; };

  };

}
#endif
