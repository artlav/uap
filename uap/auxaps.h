//#######################################################################################//           
//UAP module, Made by Artlav in 2011
//Auxillary autopilots
//#######################################################################################//
class ap_liftoff:public iautopilot{
public:
 ap_liftoff(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void init();
 void step(double simt,double simdt);
 void info();
 void reset();

 double tgt_heading,tgt_alt,rt,pitch_tgt,pitch_start,pitch_duration,off_duration,time_to_end;
 int use_target,use_grp,mode,stage;
 char mode_is[255],target[255];
 OBJHANDLE tgt_obj;
};
//#######################################################################################//
class ap_attitude:public iautopilot{
public:
 ap_attitude(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void info();

 int use_grp,mode;
 char mode_is[255];
};
//#######################################################################################//
class ap_maneuvre:public iautopilot{
public:
 ap_maneuvre(SAL *sal_in,VESSEL *vessel);
 void start(); 
 void init();
 void stop();
 void step(double simt,double simdt);
 void info();
                 
 VESSEL *tgt_ves;
 char tgt_nam[255];
 int use_grp,mode,dvdir;
 double dv,dt,level;
 char mode_is[255],dir_is[255];
};
//#######################################################################################//
class ap_tools:public iautopilot{
public:
 ap_tools(SAL *sal_in,VESSEL *vessel);
 void start();
 void step(double simt,double simdt);
 void info();
 void reset();
 void init();
                
 OBJHANDLE tgt_obj;
 char tgt_nam[255];
 int typ,stg,w_seq,param;
 DWORD key;
 double dt,ct,time_to_end,prev_alt;
 char mode_is[255];
};
//#######################################################################################//  
iautopilot *mk_maneuvre(SAL *sal_in,VESSEL *v);
iautopilot *mk_attitude(SAL *sal_in,VESSEL *v);
iautopilot *mk_liftoff(SAL *sal_in,VESSEL *v);
iautopilot *mk_tools(SAL *sal_in,VESSEL *v);
//#######################################################################################//
