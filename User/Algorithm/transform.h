#ifndef TRANSFORM_H
#define TRANSFORM_H


typedef struct{
    float q_val;                    //q轴值
    float d_val;                    //d轴值
    float alpha_val;                //alpha轴值
    float beta_val;                 //beta轴值
    float cos_val;                  //cos值
    float sin_val;                  //sin值

}Pack_Handle;

typedef struct{
    float a_val;
    float b_val;
    float c_val;
    float alpha_val;
    float beta_val;
}Clark_Handle;

void Pack_Transform(Pack_Handle *p);
void IPack_Transform(Pack_Handle *p);

void Clark_Transform(Clark_Handle *p);
void IClark_Transform(Clark_Handle *p);
#endif

