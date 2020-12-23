//#######################################################################################//              
//UAP core, Made by Artlav in 2011
//Autopilot class definition and utilitary functions
//#######################################################################################//
#pragma pack(push,1)
typedef struct{
 int typ,flags;
 double max_val,min_val;
 void *var;
 char name[255];
}var_elem;
#pragma pack (pop)

#define VT_INT 0
#define VT_DBL 1
#define VT_CHAR 2

#define VF_INP 0x00
#define VF_OUT 0x01
#define VF_DEG 0x02
#define VF_STATE 0x04  
//#######################################################################################//
class iautopilot{
public:
 iautopilot(SAL *sal_in,VESSEL *vessel);
 ~iautopilot();

 SAL *sal;
 
 void add_var(char *name,int typ,void *var,int flg,double min_val,double max_val);
 void set_dbl_var(char *name,double val);
 void set_int_var(char *name,int val);
 void set_char_var(char *name,char *val);
 virtual void step(double simt,double simdt){}
 virtual void start(){sprintf(state_string,"Started\0");}
 virtual void info(){}
 virtual void reset();
 virtual void fail(){stop();is_failed=true;}
 virtual void fail_error(char *err){sprintf(error,"%s\0",err);fail();}
 virtual void stop(){sprintf(state_string,"Stopped\0");sal->af_control_set(us,1);is_running=false;is_started=false;is_finished=true;}
 
 void    set_thgrp_level_addittive(double lv,THGROUP_TYPE g);
 double  set_thgrp_level(double lv,int groups);
 double  get_thgrp_level(int groups);
 double  give_thgrp_accel(double acc,VECTOR3 dir,int groups);
 void    stop_all_engines();
 double  get_thruster_isp(THRUSTER_HANDLE th);
 double  get_group_isp(int tg,int ismax);
 VECTOR3 get_group_thrust_vector(int tg,int ismax);
 VECTOR3 get_major_gravity_local_acc_vector();
 double  get_group_thrust_angle(int tg,int ismax);
 double  get_lin_max_accel();
 
 double get_vertical_speed();

 double get_bank(int grp=0);
 double get_pitch(int grp=0);
 double get_pitch_horizon(int grp=0);
 double get_slip_angle(int grp=0);
 double get_heading(int grp=0);
 double get_roll_heading(int grp=0);
 double get_yaw_vertical(int grp=0);
 
 void set_rotation_rcs_levels(double pi,double yi,double ri,int grp=0);
 VECTOR3 get_angular_vel_grp(int grp=0);
 void make_horvecs(VECTOR3 &wing,VECTOR3 &tail,VECTOR3 &nose,int grp=0);
 
 void att_get_specific_torque(int grp);
 void att_guide(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp);
 void att_control(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp);
 void att_control_exp(double dt,double pitch,double yaw,double roll,double tgt_pitch,double tgt_yaw,double tgt_roll,int flg,int grp);
 void att_killrot(int flg,int grp);
 
 void asf_get_specific_torque(int grp);
 void asf_guide(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp);
 void asf_control(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp);
 void asf_control_exp(double dt,double pitch,double yaw,double roll,double tgt_pitch,double tgt_yaw,double tgt_roll,int flg,int grp);
 void asf_zero();
 
 int is_started,is_running,is_finished,is_loaded,is_failed;
 VESSEL *us;
 char name[255],id[255],state_string[255],error[255];
 int var_cnt;
 double max_time_accel;
 var_elem *vars;
 
 double kp_pitch,kp_yaw,kp_roll,kd_pitch,kd_yaw,kd_roll;
 double delta_cmd_pitch,delta_cmd_yaw,delta_cmd_roll;
 double target_pitch,target_yaw,target_roll,last_target_pitch,last_target_yaw,last_target_roll;
 VECTOR3 spec_torque,rot_vel_bias;
 int att_guide_not_set,att_split;
};
//#######################################################################################//

