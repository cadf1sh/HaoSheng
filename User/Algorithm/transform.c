/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 坐标系变换
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "transform.h"

void Pack_Transform(Pack_Handle *p){
    p->d_val = p->alpha_val * p->cos_val + p->beta_val * p->sin_val;
    p->q_val = p->beta_val * p->cos_val - p->alpha_val * p->sin_val;
}

void IPack_Transform(Pack_Handle *p){
    p->alpha_val = p->d_val * p->cos_val - p->q_val * p->sin_val;
    p->beta_val = p->q_val * p->cos_val + p->d_val * p->sin_val;
}

void Clark_Transform(Clark_Handle *p){
    p->alpha_val = p->a_val;
    //p->beta_val = 1.0f / sqrtf(3) * p->a_val + 2.0f / sqrt(3) * p->b_val;
    p->beta_val = 0.577350269f * p->a_val + 1.154700538f * p->b_val;
}

void IClark_Transform(Clark_Handle *p){
    p->a_val = p->alpha_val;
    //p->b_val = -0.5f * p->alpha_val + sqrtf(3) / 2.0f * p->beta_val;
    p->b_val = -0.5f * p->alpha_val + 0.8660254f * p->beta_val;
    //p->c_val = -0.5f * p->alpha_val - sqrtf(3) / 2.0f * p->beta_val;
    p->c_val = -0.5f * p->alpha_val - 0.8660254f * p->beta_val;
}


