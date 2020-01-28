#include "main.h"
#include "hard_config.h"
#include <avr/io.h>


 /*
 Инициализации адф функцией init_adf() не происходит -
 в ней происходит инициализация массивов

*/   



//-------------------------
#define RF2R 0
#define RF2AB 1
#define RF1R 2
#define RF1AB 3
#define R_COUNT fkvarz/25000
//unsigned char t1,t2,t3;


//------------------------
typedef union {
struct{
unsigned char C:2;
unsigned int R:14;
unsigned char DB16:1;
unsigned char DB17:1;
unsigned char DB18:1;
unsigned char DB19:1; 
unsigned char DB20:1;
unsigned char DB21:1;
}st;
unsigned char arr[3] ;
}unR;
//-----------------------
typedef union {
struct{
unsigned char C:2;
unsigned char A:6;
 unsigned char DB8:1;
unsigned int B:11; 
unsigned char DB20:1;
unsigned char DB21:1;
}st;
unsigned char arr[3] ;
}unAB;
//-----------------------
//-----------------------
unR R;
unAB AB;
//-----------------------



//---------------------------------------------------------------------
void send_bt(unsigned char bt,unsigned char maska,unsigned char count)
{
do{
if (bt&maska)
    {
    CDATA_1();
    }
    else
    {
    CDATA_0();
    }
    CCLOCK_1();
    maska>>=1;
   CCLOCK_0();
}while (--count);
}
//------------------------------------------------
void out_paket_R(unsigned char *massiv)

{
LE_0();
send_bt(massiv[2],0x20,6);
send_bt(massiv[1],0x80,8);
send_bt(massiv[0],0x80,8);
    CDATA_0();
LE_1();    
}
//-----------------------------------------
void out_paket_AB(unsigned char *massiv)

{
LE_0();
send_bt(massiv[2],0x20,6);
send_bt(massiv[1],0x80,8);
send_bt(massiv[0],0x80,8);
    CDATA_0();
LE_1();    
}
//--------------------------------------
void adf_init(void)
{
LE_1();   
R.st.DB21=1; //передача
R.st.DB20=0;
R.st.DB19=0; // нормальный режим
R.st.DB18=0; // !4 милиампера
R.st.DB17=1; // positive
R.st.R=R_COUNT;
//--------------------------------------
AB.st.DB21=0;// включаем
AB.st.DB20=1; // р=32
AB.st.B=0;
AB.st.A=0;
}
void out_fr(unsigned long freg, unsigned char kanal)
// частота в килогерцах, канал rf1 - 1 rf2 - 2
{
//unsigned long B;
unsigned int a_;
unsigned long oll;

unsigned int b_;
unsigned int p;

if (freg <1800000ul)
{

p=32;
AB.st.DB20=1; // р=32
}
else
{

p=64;
AB.st.DB20=0; // р=64
}


if (kanal==1)    // RF1
{
R.st.C=RF1R; // RF1
R.st.DB21=1; //передача
R.st.DB20=0;//x
 // -------------------------------------------------------------



// R_COUNT  был вычислен при инициализации и не меняется

 out_paket_R(R.arr); //для 4 мег кварца  = 160
// добавляем недостающие биты к пакету
R.st.C=RF2R; // RF2
R.st.DB21=0;// RF1 reference divider
R.st.DB20=0;// 
out_paket_R(R.arr);// передаем пакет RF2
// теперь вычисляем значение 
//
oll= (unsigned long)freg/(unsigned long)25;  // общий счетчик

b_=oll/p;
a_=oll - (b_*p);
 AB.st.B=(unsigned int)b_;// B;// береженого бог бережет 

 AB.st.A=(unsigned int)a_;

 AB.st.C=RF1AB;
 out_paket_AB(AB.arr);
 }
 else
 {
  R.st.C=RF2R;
 R.st.DB21=0; //RF2 referense divider output
R.st.DB20=0;//
//R.st.DB19=0; // нормальный режим
//R.st.DB18=1; // 4 милиампера
//R.st.DB17=1; // positive//
// не трогаем, они установлены при инициализации
// как и    R_COUNT
 out_paket_R(R.arr);
 // передали пакет, добавляем недостающие биты в 1 пакете
 // Р12 Р11  обнуляем
   R.st.C=RF1R;
 R.st.DB21=0; //Р12
R.st.DB20=0;// Р11
 out_paket_R(R.arr); // передаем пакет RF1
oll= (unsigned long)freg/(unsigned long)25;  // общий счетчик

b_=oll/p;
a_=oll - (long)(b_*p);

 AB.st.B=(unsigned int)b_;// B;// береженого бог бережет 

 AB.st.A=(unsigned int)a_;
 AB.st.C=RF2AB;
 out_paket_AB(AB.arr);
 }

 
 }
 //--------------------------------------------------
 void in_fr(unsigned char kan,unsigned int kdel)
 {
 unsigned int a_,b_;

 //delay_ms(4);
 if (kan==1)
 {
 R.st.C=RF1R; // RF1
R.st.DB21=1; //
R.st.DB20=0;//
 out_paket_R(R.arr);
 R.st.DB21=0; //
R.st.DB20=1;//
  R.st.C=RF2R; // RF2
  out_paket_R(R.arr);
 AB.st.DB20=1; // р=32
 b_=kdel/32;
 a_=kdel-(b_*32);
  AB.st.B=(unsigned int) b_;// B;// береженого бог бережет 
 AB.st.A=(unsigned int)a_;
 AB.st.C=RF1AB;
 out_paket_AB(AB.arr); 
 }
 else
 {
 R.st.C=RF2R; // RF2
R.st.DB21=1; //
R.st.DB20=1;//
out_paket_R(R.arr);
 R.st.DB21=0; //
R.st.DB20=0;//
  R.st.C=RF1R; // RF1
  out_paket_R(R.arr);
  AB.st.DB20=1; // р=32
 b_=kdel/32;
 a_=kdel-(b_*32);
  AB.st.B=(unsigned int) b_;// B;// береженого бог бережет 
 AB.st.A=(unsigned int)a_;
 AB.st.C=RF2AB;
 out_paket_AB(AB.arr);  
 
 }  

 }
 
//--------------------------------------
//--------------------------------------
