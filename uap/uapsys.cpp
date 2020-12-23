//#######################################################################################//
//UAP system, Made by Artlav in 2011
//The UAP core itself
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE 1
//#######################################################################################//
//Added align planes autopilot
//Added target to lift-off autopilot
//Fixed overspin in hover heading
//
//No time accel limits button
//Bug: Lunar man, del all, add 2 attitudes, only in release.
//Docking, something
//Add MFD scroll list
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE 1
#include <string.h>
#include <malloc.h>
#include <stdlib.h>
#include <math.h>
#include "sal.h"
#include "sal_imp.h"
#include "iap.h"
#define _uapsys
#include "uapsys.h"  
//#######################################################################################//
#include "peg.h" 
#include "auxaps.h" 
#include "dock.h" 
#include "land.h" 
#include "orbital.h" 
#include "air.h"                                                                                            
//#######################################################################################//
SAL *the_sal=NULL;
//#######################################################################################//
int new_lib(char *id,mk_func make)
{
 int i,c;
 c=-1;
 for(i=0;i<lib_cnt;i++)if(!libs[i].used){c=i;break;}
 if(c==-1){
  c=lib_cnt;
  libs=(lib_elem*)realloc(libs,sizeof(lib_elem)*(c+1));
  if(!libs)return -1;
  lib_cnt=c+1;
 }
 libs[c].used=1;
 libs[c].id=id; //Is that ok?
 libs[c].make=make;
 return c;
} 
//#######################################################################################//
iautopilot *get_lib_obj(char *id,VESSEL *v)
{
 int i;
 for(i=0;i<lib_cnt;i++)if(libs[i].used)if(!strcmp(libs[i].id,id)){
  iautopilot *res=libs[i].make(the_sal,v);
  sprintf(res->id,"%s",id);
  return res;
 }
 return NULL;
} 
//#######################################################################################//
//#######################################################################################//
int new_seq(VESSEL *v)
{
 int i,c;
 c=-1;
 for(i=0;i<seq_cnt;i++)if(!seqs[i].used){c=i;break;}
 if(c==-1){
  c=seq_cnt;
  seqs=(uap_seq*)realloc(seqs,sizeof(uap_seq)*(c*2+1));
  if(!seqs)return -1;
  seq_cnt=c*2+1;
  for(i=c;i<seq_cnt;i++)seqs[i].used=0;
 }
 seqs[c].used=1;
 seqs[c].n=c;
 seqs[c].run=0;
 seqs[c].us=v;
 seqs[c].cnt=0;
 seqs[c].cur=-1;
 seqs[c].seq=NULL;
 return c;
}
//#######################################################################################//
void del_seq(int c)
{
 int i;
 for(i=0;i<seqs[c].cnt;i++)delete seqs[c].seq[i].ap;
 free(seqs[c].seq);
 seqs[c].used=0;
 seqs[c].run=0;
 seqs[c].us=0;
 seqs[c].cnt=0;
 seqs[c].cur=-1;
 seqs[c].seq=NULL;
}
//#######################################################################################//
int get_seq_by_vessel(VESSEL *v)
{
 int i;
 for(i=0;i<seq_cnt;i++)if(seqs[i].used)if(seqs[i].us==v)return i;
 return -1;
}
//#######################################################################################//
int add_iap_to_seq(int n,iautopilot *ap)
{
 int c;
 c=seqs[n].cnt;
 seqs[n].seq=(uap_seq_elem*)realloc(seqs[n].seq,sizeof(uap_seq_elem)*(c+1));
 if(!seqs[n].seq)return -1;
 seqs[n].cnt=seqs[n].cnt+1;
 seqs[n].seq[c].ap=ap;
 if(c==0)seqs[n].cur=0;
 return c;
}
//#######################################################################################//
//#######################################################################################//
DLLEXPORT void uap_stop_seq(int n)
{
 if(n!=-1)if(seqs[n].run){
  seqs[n].seq[seqs[n].cur].ap->stop();
  seqs[n].seq[seqs[n].cur].ap->is_finished=0;
  seqs[n].cur=0;
  seqs[n].run=0;
 }
}
//#######################################################################################//
DLLEXPORT void uap_start_seq(int n)
{
 uap_clear_error();
 if(n!=-1)if(!seqs[n].run){
  seqs[n].cur=0;
  seqs[n].run=1;
  seqs[n].seq[0].ap->reset();
 }
}
//#######################################################################################//
DLLEXPORT void uap_del_seq_step(int &n,int &p)
{
 int i;
 if(n==-1)return;
 if(seqs[n].cnt<=1){
  del_seq(n);
  n=-1;
 }else{
  delete seqs[n].seq[p].ap;
  for(i=p;i<seqs[n].cnt-1;i++)seqs[n].seq[i]=seqs[n].seq[i+1];
  seqs[n].cnt--;
  if(p>=seqs[n].cnt)p=seqs[n].cnt-1;
 }
}
//#######################################################################################//
DLLEXPORT void uap_move_seq_step(int n,int &p,int tp)
{  
 uap_seq_elem tmp;
 
 if(n==-1)return;
 if(tp==0){
  if(p<=0)return;
  tmp=seqs[n].seq[p-1];
  seqs[n].seq[p-1]=seqs[n].seq[p];
  seqs[n].seq[p]=tmp;
  p--;
 }
 if(tp==1){
  if(p>=seqs[n].cnt-1)return;
  tmp=seqs[n].seq[p+1];
  seqs[n].seq[p+1]=seqs[n].seq[p];
  seqs[n].seq[p]=tmp;
  p++;
 }
}
//#######################################################################################//
void uap_add_seq_ap(int &n,int &p,VESSEL *ves,iautopilot *ap)
{
 uap_seq_elem tmp;
 int i;
 
 if(n==-1)n=new_seq(ves);
 add_iap_to_seq(n,ap);
 if(seqs[n].cnt==0){
  seqs[n].cur=0;
  p=0;
 }else if(p==seqs[n].cnt-2){
  p=seqs[n].cnt-1;
 }else{
  tmp=seqs[n].seq[seqs[n].cnt-1];
  for(i=seqs[n].cnt-1;i>p+1;i--)seqs[n].seq[i]=seqs[n].seq[i-1];
  seqs[n].seq[p+1]=tmp;
  p++;
 }
 if(p>=seqs[n].cnt)p=seqs[n].cnt-1;
}
//#######################################################################################//
//#######################################################################################//
int const_test(char *val)
{
 if(!strcmp(val,"main"))return xTHGROUP_MAIN;
 if(!strcmp(val,"hover"))return xTHGROUP_HOVER;
 if(!strcmp(val,"retro"))return xTHGROUP_RETRO;
 if(!strcmp(val,"maininv"))return xTHGROUP_MAIN_INV;
 
 if((!strcmp(val,"g"))||(!strcmp(val,"G")))return OAPI_KEY_G;
 if((!strcmp(val,"k"))||(!strcmp(val,"K")))return OAPI_KEY_K;
 if((!strcmp(val,"j"))||(!strcmp(val,"J")))return OAPI_KEY_J;
 if((!strcmp(val,"f"))||(!strcmp(val,"F")))return OAPI_KEY_F;
 if((!strcmp(val,"e"))||(!strcmp(val,"E")))return OAPI_KEY_E;
 if((!strcmp(val,"o"))||(!strcmp(val,"O")))return OAPI_KEY_O;
 return atoi(val);
}
//#######################################################################################//
int parse_seq_def(char *sdin,VESSEL *ves,int cur)
{
 int i,len,n;
 char nam[255],val[255],sd[2550];
 iautopilot *ap;
 
 n=new_seq(ves);
 sprintf(sd,"%s\0",sdin);
 
 l3:
  len=strlen(sd);
  if(sd[0]=='.')goto last;
  for(i=0;i<len;i++)if(sd[i]!='(')nam[i]=sd[i];else{nam[i]=0;strcpy(sd,&sd[i+1]);break;}
  if(!(ap=get_lib_obj(nam,ves)))return -1;
  if(sd[0]==')')goto l1;
  l2:
   val[0]=0;
   nam[0]=0;
   for(i=0;i<len;i++)if(sd[i]!='=')nam[i]=sd[i];else{nam[i]=0;strcpy(sd,&sd[i+1]);break;}
   for(i=0;i<len;i++)if((sd[i]!=',')&&(sd[i]!=')'))val[i]=sd[i];else{val[i]=0;strcpy(sd,&sd[i]);break;}
   
   for(i=0;i<ap->var_cnt;i++)if(!strcmp(ap->vars[i].name,nam)){
    switch(ap->vars[i].typ){
     case VT_DBL:{
      double par=strtod(val,NULL);
      if(ap->vars[i].flags&VF_DEG)par=par*RAD;
      ap->set_dbl_var(nam,par);
      break;
     }
     case VT_INT:ap->set_int_var(nam,const_test(val));break;
     case VT_CHAR:ap->set_char_var(nam,val);break;
     default:return -1;
    }
    break;
   }  
   if(sd[0]==','){strcpy(sd,&sd[1]);goto l2;}
  l1:
   add_iap_to_seq(n,ap);
   strcpy(sd,&sd[1]);
   len=strlen(sd);
   for(i=0;i<len;i++){
    if(sd[i]=='.')goto last;
    if((sd[i]==',')||(sd[i]==' ')||(sd[i]=='\t'))continue;
    strcpy(sd,&sd[i]);
    break;
   }
   goto l3;
   
 last:
  seqs[n].cur=cur;
  if(cur!=-1){
   seqs[n].run=true;
   seqs[n].seq[cur].ap->is_started=1;
   seqs[n].seq[cur].ap->is_running=1;
  }
  return n;
}
//#######################################################################################//
DLLEXPORT void uap_save(FILEHANDLE scn)
{
 int i,j,k;
 char tmp[2550],v[2550],r[2550];
 
 for(j=0;j<seq_cnt;j++)if(seqs[j].used)if(seqs[j].cur<seqs[j].cnt){
  if((seqs[j].cur==-1)||(!seqs[j].run))sprintf(r,"$%s:",the_sal->get_ves_name(seqs[j].us));
                                  else sprintf(r,">%s:",the_sal->get_ves_name(seqs[j].us));
  for(k=max(seqs[j].cur,0);k<seqs[j].cnt;k++){
   iautopilot *ap=seqs[j].seq[k].ap;
   sprintf(tmp,"%s(",ap->id);
   strcat(r,tmp);
   
   for(i=0;i<ap->var_cnt;i++){
    switch(ap->vars[i].typ){
     case VT_DBL:sprintf(v,"%f",ap->vars[i].flags&VF_DEG?*((double*)ap->vars[i].var)/RAD:*((double*)ap->vars[i].var));break;
     case VT_INT:{
      if(!strcmp(ap->vars[i].name,"engine"))switch(*((int*)ap->vars[i].var)){
       case 1:sprintf(v,"main");break;
       case 2:sprintf(v,"retro");break;
       case 4:sprintf(v,"hover");break;
       case 8:sprintf(v,"maininv");break;
       default:sprintf(v,"%d",*((int*)ap->vars[i].var));
      }else if((!strcmp(ap->vars[i].name,"key"))||(!strcmp(ap->vars[i].name,"gear_key")))switch(*((int*)ap->vars[i].var)){
       case OAPI_KEY_G:sprintf(v,"g");break;
       case OAPI_KEY_J:sprintf(v,"j");break;
       case OAPI_KEY_K:sprintf(v,"k");break;
       case OAPI_KEY_F:sprintf(v,"f");break;
       case OAPI_KEY_E:sprintf(v,"e");break;
       case OAPI_KEY_O:sprintf(v,"o");break;
       default:sprintf(v,"%d",*((int*)ap->vars[i].var));
      }else sprintf(v,"%d",*((int*)ap->vars[i].var));
      break;
     }
     case VT_CHAR:sprintf(v,"%s",(char*)ap->vars[i].var);break;
    }
    if(ap->vars[i].flags&VF_OUT){
     if(i==ap->var_cnt-1)sprintf(tmp,")");
                    else sprintf(tmp,"");
    }else{
     if(i!=ap->var_cnt-1){
      if(ap->vars[i+1].flags&VF_OUT)sprintf(tmp,"%s=%s",ap->vars[i].name,v);
                               else sprintf(tmp,"%s=%s,",ap->vars[i].name,v);
     }else sprintf(tmp,"%s=%s)",ap->vars[i].name,v);
    }                   
    strcat(r,tmp);
   }
   if(ap->var_cnt==0)strcat(r,")");
   if(k==seqs[j].cnt-1)strcat(r,".");else strcat(r,",\n  ");
  }
  the_sal->write_scn_line(scn,r,"");
  r[0]=0;
 }
}
//#######################################################################################//
DLLEXPORT void uap_load(FILEHANDLE scn)
{
 char *line,par[2550],ves[255];
 int i,len,md,o,st;
 VESSEL *v;
 
 md=0;
 o=0;
 st=-1;
 while(the_sal->read_scn_line(scn,line)){
  len=strlen(line);
  if((md==1)&&((line[0]=='$')||(line[0]=='>'))){
   o=0;
   md=0;
   if(v=the_sal->vessel_by_name(ves))parse_seq_def(par,v,st);
  }
  if(md==0){
   if(line[0]=='$')st=-1;
   if(line[0]=='>')st=0;
   for(i=1;i<len;i++)if(line[i]!=':')ves[i-1]=line[i];else{
    ves[i-1]=0;
    strcpy(par,&line[i+1]);
    o=len-i-1;
    md=1;
    break;
   }
  }else{
   strcpy(&par[o],&line[0]);
   o+=len;
  }  
 }
 if(md==1)if(v=the_sal->vessel_by_name(ves))parse_seq_def(par,v,st);
}
//#######################################################################################//
DLLEXPORT void uap_step(double SimT,double SimDT,double mjd)
{
 int i,u;
 double ta;

 ta=the_sal->get_time_accel();

 if(uap_time_acc==2)for(i=0;i<seq_cnt;i++)if(seqs[i].used)if(seqs[i].run){
  if(ta>uap_prev_time_accel)uap_set_time_accel=ta;
  if((ta<uap_prev_time_accel)&&(ta>=1))uap_set_time_accel=ta;
  break;
 }

 u=0;
 if(uap_time_acc==2)if(ta<uap_set_time_accel)ta=uap_set_time_accel;
    
 for(i=0;i<seq_cnt;i++)if(seqs[i].used)if(seqs[i].run){
  if((seqs[i].cur!=-1)&&(seqs[i].cur<seqs[i].cnt)){
   if(seqs[i].seq[seqs[i].cur].ap->is_started){   
    seqs[i].seq[seqs[i].cur].ap->step(SimT,SimDT);
   }else if(seqs[i].seq[seqs[i].cur].ap->is_finished){
    if(seqs[i].seq[seqs[i].cur].ap->is_failed){
     sprintf(uap_error,"%s\0",seqs[i].seq[seqs[i].cur].ap->error);
     uap_fail=1;
     seqs[i].run=0;
     seqs[i].cur=0;
     continue;
    }
    seqs[i].seq[seqs[i].cur].ap->reset();
    seqs[i].cur++;
    if(seqs[i].cur>=seqs[i].cnt){seqs[i].run=0;seqs[i].cur=0;}
                           else seqs[i].seq[seqs[i].cur].ap->start();
   }else seqs[i].seq[seqs[i].cur].ap->start(); 
   if(seqs[i].run)if(uap_time_acc){
    u=1;
    if(ta>seqs[i].seq[seqs[i].cur].ap->max_time_accel)ta=seqs[i].seq[seqs[i].cur].ap->max_time_accel;
   }
  }else{
   seqs[i].cur=0;
   if(seqs[i].cnt==0)seqs[i].run=0;
   seqs[i].seq[seqs[i].cur].ap->start();
  }
 }
 
 if(u)the_sal->set_time_accel(ta);
 uap_prev_time_accel=the_sal->get_time_accel();  
}
//#######################################################################################//
void test_seq_make()
{
 uap_clear_error();
 int n;
 //n=parse_seq_def("trans_orbit(engine=main,apoapsis=200000,periapsis=200000,ta=0,azimuth=90,pitch=40,pitch_time=130).",get_focus_ves(),-1);
 //n=parse_seq_def("trans_orbit(engine=main,apoapsis=200000,periapsis=200000,ta=0,azimuth=90,pitch=30,pitch_time=130).",get_focus_ves(),-1);
 
 //n=parse_seq_def("lift_off(engine=hover,heading=70,altitude=60), tools(type=0,key=g),trans_orbit(engine=main,apoapsis=20000,periapsis=20000,ta=0,target=GL-TGT,pitch=30,pitch_time=10).",get_focus_ves(),0);
 //n=parse_seq_def("get_on_pad(engine=hover,tgt_lat=41.131593,tgt_lon=-33.4375).",get_focus_ves(),0);
 
 //n=parse_seq_def("lift_off(engine=hover,heading=70,altitude=40),get_on_pad(engine=hover,tgt_lat=41.121702,tgt_lon=-33.429921),lift_off(engine=hover,heading=70,altitude=40),get_on_pad(engine=hover,tgt_lat=41.118407,tgt_lon=-33.437500),lift_off(engine=hover,heading=70,altitude=100),get_on_pad(engine=hover,tgt_lat=41.131593,tgt_lon=-33.4375).",get_focus_ves(),0);
 
 //n=parse_seq_def("get_on_pad(engine=hover).",get_focus_ves(),0);
 
 //n=parse_seq_def("lift_off(mode=0,engine=hover,heading=70),tools(type=0,key=g),lift_off(mode=2,engine=main,heading=70,pitch_tgt=40,pitch_duration=10,off_duration=0),trans_orbit(engine=main,apoapsis=5000,periapsis=5000,ta=0,azimuth=70,pitch=30,pitch_time=10),tools(type=1,wait=6405),tools(type=0,key=g),get_on_pad(engine=hover,tgt_lat=41.131593,tgt_lon=-33.4375).",get_focus_ves(),0);
 
 //n=parse_seq_def("lift_off(engine=hover,heading=70,altitude=120), tools(type=0,key=g),trans_orbit(engine=hover,apoapsis=20000,periapsis=20000,ta=0,azimuth=70,pitch=30,pitch_time=10).",get_focus_ves(),0);
 //n=parse_seq_def("lift_off(engine=hover,heading=70,altitude=120), tools(type=0,key=g),trans_orbit(engine=retro,apoapsis=20000,periapsis=20000,ta=0,azimuth=250,pitch=70,pitch_time=15).",get_focus_ves(),0);

 //n=parse_seq_def("lift_off(mode=2,engine=hover,heading=70,pitch_tgt=30,pitch_duration=10,off_duration=2), tools(type=0,key=g),trans_orbit(engine=hover,apoapsis=20000,periapsis=20000,ta=0,azimuth=70).",get_focus_ves(),0); 
 //n=parse_seq_def("lift_off(mode=2,engine=main,heading=90,pitch_tgt=40,pitch_duration=130,off_duration=10),trans_orbit(engine=main,apoapsis=200000,periapsis=200000,ta=0,azimuth=90).",get_focus_ves(),0);
 //n=parse_seq_def("lift_off(mode=0,engine=hover,heading=70),tools(type=0,key=g),lift_off(mode=2,engine=main,heading=70,pitch_tgt=40,pitch_duration=10,off_duration=0),trans_orbit(engine=main,apoapsis=20000,periapsis=20000,ta=0,azimuth=70).",get_focus_ves(),0);
 //n=parse_seq_def("lift_off(mode=0,engine=hover,heading=70),tools(type=0,key=g),lift_off(mode=2,engine=main,heading=70,pitch_tgt=30,pitch_duration=10,off_duration=0),trans_orbit(engine=main,apoapsis=20000,periapsis=20000,ta=0,azimuth=70),tools(type=1,wait=10),trans_orbit(engine=main,kind=1,apoapsis=1000000,ta=0),tools(type=2),attitude(mode=0),trans_orbit(engine=main,kind=1,apoapsis=1100000).",get_focus_ves(),0);
 //n=parse_seq_def("lift_off(mode=0,engine=hover,heading=70),tools(type=0,key=g),lift_off(mode=2,engine=main,heading=70,pitch_tgt=30,pitch_duration=10,off_duration=0),trans_orbit(engine=main,apoapsis=20000,periapsis=20000,ta=0,azimuth=70),attitude(mode=2),tools(type=1,wait=10),trans_orbit(engine=main,kind=1,apoapsis=1000000,ta=0),attitude(mode=2),tools(type=2),attitude(mode=0),attitude(mode=2),trans_orbit(engine=main,kind=1,apoapsis=1100000),attitude(mode=2).",get_focus_ves(),0);
 //n=parse_seq_def("runway_off(engine=main,altitude=1000,heading=145,v1=170,gear_key=g),lift_off(engine=main,mode=2,heading=145,altitude=491,pitch_tgt=60,pitch_duration=130,off_duration=1,stage=0),trans_orbit(engine=main,kind=0,apoapsis=200000,periapsis=200000,ta=0,azimuth=145,mode=0).",get_focus_ves(),0); 
 //n=parse_seq_def("dock(target=ISS,port=0,with_port=0).",get_focus_ves(),-1);
 //n=parse_seq_def("runway_off(engine=main,altitude=1000,heading=340,v1=150,gear_key=g),air_hold(engine=main,heading=340,altitude=1000,velocity=220,alt_rate=100,vel_rate=-5,hdg_rate=5,onoff=7).",get_focus_ves(),0); 
 //n=parse_seq_def("air_hold(engine=main,heading=340,altitude=1000,velocity=220,alt_rate=1,vel_rate=-0.1,hdg_rate=0.1,onoff=7).",get_focus_ves(),0); 

 //n=parse_seq_def("align(target=Luna-OB1).",get_focus_ves(),-1);
 //n=parse_seq_def("lift_off(engine=hover,mode=0,heading=90,altitude=1500,off_duration=1),tools(type=0,key=g),lift_off(engine=main,mode=2,heading=90,pitch_tgt=45,pitch_duration=10,off_duration=1),trans_orbit(engine=main,kind=0,apoapsis=200000,periapsis=200000,ta=0,mode=0).",get_focus_ves(),-1); 

 //n=parse_seq_def("approach(engine=main,target=ISS,max_velocity=40,tgt_distance=1500),tools(type=0,key=k),dock(target=ISS,port=0,with_port=0).",get_focus_ves(),-1); 
 //n=parse_seq_def("sync_orbit(engine=main,target=ISS).",get_focus_ves(),-1); 

 //n=parse_seq_def("lift_off(mode=0,engine=hover,heading=50,altitude=3000),lift_off(mode=2,engine=main,heading=50,pitch_tgt=40,pitch_duration=5,off_duration=0),trans_orbit(engine=main,apoapsis=20000,periapsis=20000,ta=0).",get_focus_ves(),0);
 //n=parse_seq_def("attitude(mode=0).",get_focus_ves(),0);
 n=parse_seq_def("hohmann(engine=main,target=Moon,tgt_orbit_radius=500000).",the_sal->get_focus_ves(),0);
 
 if(n!=-1)seqs[n].run=0;
}
//#######################################################################################//
void uap_clear_error()
{        
 sprintf(uap_error,"All is Ok?\0");
 uap_fail=0;
}
//#######################################################################################//
void uap_init()
{
 seq_cnt=0;
 seqs=NULL;
 lib_cnt=0;
 libs=NULL;
 uap_time_acc=1;
 uap_set_time_accel=1;
 uap_prev_time_accel=1;
 uap_clear_error();

 the_sal=new SAL_IMP();
 
 new_lib("lift_off",mk_liftoff);
 new_lib("runway_off",mk_runway_off);
 new_lib("air_hold",mk_air_hold);
 new_lib("trans_orbit",mk_transorbit);
 new_lib("dock",mk_dock);
 new_lib("get_on_pad",mk_get_on_pad);
 new_lib("align",mk_align);
 new_lib("sync_orbit",mk_sync_orbit);
 new_lib("hohmann",mk_hohmann);
 new_lib("approach",mk_approach);
 new_lib("attitude",mk_attitude);
 new_lib("maneuvre",mk_maneuvre);
 new_lib("tools",mk_tools);
}
//#######################################################################################//
DLLEXPORT void uap_reset()
{
 int i;
 for(i=0;i<seq_cnt;i++)if(seqs[i].used)del_seq(i);
 
 free(seqs);
 seqs=NULL;
 seq_cnt=0;
 uap_time_acc=1;
 uap_set_time_accel=1;
 uap_prev_time_accel=1;
}
//#######################################################################################//
DLLEXPORT void uapd_init()
{
 uap_init();
}
//#######################################################################################//
DLLEXPORT void uapd_set_seq(char *ves,char *seq)
{
 parse_seq_def(seq,the_sal->vessel_by_name(ves),-1);
}
//#######################################################################################//
DLLEXPORT void uapd_seq_state(char *ves,int state)
{
 int n=get_seq_by_vessel(the_sal->vessel_by_name(ves));
 if(n!=-1){
  if(state==1)uap_start_seq(n);
  if(state==0)uap_stop_seq(n);
 }
}
//#######################################################################################//
DLLEXPORT uap_seq *uapd_get_ves_seq(char *ves)
{
 int n=get_seq_by_vessel(the_sal->vessel_by_name(ves));
 if(n==-1)return NULL;
 return &seqs[n];
}
//#######################################################################################//
DLLEXPORT void uapd_add_seq_ap(uap_seq *seq,char *id,int &p)
{
 if(!seq)seq=&seqs[new_seq(the_sal->get_focus_ves())];
 iautopilot *ap=get_lib_obj(id,seq->us);
 if(ap)uap_add_seq_ap(seq->n,p,seq->us,ap);
}
//#######################################################################################//
DLLEXPORT var_elem *uapd_get_seq_vars(uap_seq *seq,int n,int &cnt)
{
 if(!seq)return NULL;
 seq->seq[n].ap->info();
 cnt=seq->seq[n].ap->var_cnt;
 return seq->seq[n].ap->vars;
}
//#######################################################################################//
DLLEXPORT void uapd_set_seq_var_string(uap_seq *seq,int n,int v,char *st)
{
 if(!seq)return;
 seq->seq[n].ap->set_char_var(seq->seq[n].ap->vars[v].name,st);
}
//#######################################################################################//
DLLEXPORT lib_elem *uapd_get_libs(int &cnt)
{
 cnt=lib_cnt;
 return libs;
}
//#######################################################################################//
DLLEXPORT char *uapd_get_seqelem_name(uap_seq *seq,int n)
{
 if(!seq)return "";
 if((n<0)||(n>=seq->cnt))return "";
 return seq->seq[n].ap->name;
}
//#######################################################################################//