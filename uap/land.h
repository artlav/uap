//#######################################################################################//
//UAP module, Made by Artlav in 2011
//Landing autopilot
//#######################################################################################//
class ap_get_on_pad:public iautopilot{
public:
 ap_get_on_pad(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();

 int use_grp;
 double tgt_lat,tgt_lon;
 
 VECTOR3 tgt_pos;
 int stage;
 double dist,set_a,set_ns,tgt_rad,set_ang;
};
//#######################################################################################//
iautopilot *mk_get_on_pad(SAL *sal_in,VESSEL *v);
//#######################################################################################//
