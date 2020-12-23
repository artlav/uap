//#######################################################################################//
//UAP system, Made by Artlav in 2011
//The UAP core itself
//#######################################################################################//
#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif
//#######################################################################################//
typedef struct{
 iautopilot *ap;
}uap_seq_elem;

#pragma pack(push, 1)
typedef struct{
 int used,run;
 VESSEL *us;
 int cnt,cur,n;
 uap_seq_elem *seq;
}uap_seq;
#pragma pack (pop)
//#######################################################################################//
typedef iautopilot *mk_func(SAL *,VESSEL*);
#pragma pack(push, 1)
typedef struct{
 int used;
 char *id;
 mk_func *make;
}lib_elem;
#pragma pack (pop)
//#######################################################################################//
#ifdef _uapsys
int seq_cnt;
uap_seq *seqs;
int lib_cnt;
lib_elem *libs;
int uap_time_acc;    
int uap_fail;
char uap_error[255];
double uap_set_time_accel;
double uap_prev_time_accel;
#else
extern int seq_cnt;
extern uap_seq *seqs;
extern int lib_cnt;
extern lib_elem *libs;
extern int uap_time_acc;
extern int uap_fail;
extern char uap_error[255];
extern double uap_set_time_accel;
extern double uap_prev_time_accel;
#endif
//#######################################################################################//
int new_lib(char *id,mk_func make);
iautopilot *get_lib_obj(char *id,VESSEL *v);
int new_seq(VESSEL *v);
void del_seq(int c);
int get_seq_by_vessel(VESSEL *v);
int add_iap_to_seq(int n,iautopilot *ap);
void uap_add_seq_ap(int &n,int &p,VESSEL *ves,iautopilot *ap);

int const_test(char *val);
int parse_seq_def(char *sdin,VESSEL *ves,int cur);
void test_seq_make();
void uap_init();   
void uap_clear_error();
//#######################################################################################//
extern "C" {
 DLLEXPORT void uap_stop_seq(int n);
 DLLEXPORT void uap_start_seq(int n);
 DLLEXPORT void uap_del_seq_step(int &n,int &p);
 DLLEXPORT void uap_move_seq_step(int n,int &p,int tp);
 DLLEXPORT void uap_save(FILEHANDLE scn);
 DLLEXPORT void uap_load(FILEHANDLE scn);
 DLLEXPORT void uap_step(double SimT,double SimDT,double mjd);
 DLLEXPORT void uap_reset();
 DLLEXPORT void uapd_init();
 DLLEXPORT void uapd_set_seq(char *ves,char *seq);
 DLLEXPORT void uapd_seq_state(char *ves,int state);
 DLLEXPORT uap_seq *uapd_get_ves_seq(char *ves);
 DLLEXPORT void uapd_add_seq_ap(uap_seq *seq,char *id,int &p);
 DLLEXPORT var_elem *uapd_get_seq_vars(uap_seq *seq,int n,int &cnt);
 DLLEXPORT void uapd_set_seq_var_string(uap_seq *seq,int n,int v,char *st);
 DLLEXPORT lib_elem *uapd_get_libs(int &cnt);
 DLLEXPORT char *uapd_get_seqelem_name(uap_seq *seq,int n);
}