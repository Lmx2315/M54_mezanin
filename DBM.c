



unsigned int convert_to_volt(int a)

{
u16 i=0;
int a_min=10;
int a_max=0;


 for (i=0;i<330;i++) 
 {
    if (a>DBm[i]) a_min=i;
    if (a<DBm[i]) a_max=i;
 }

 if (a_max==0) {a_max=330*(a/8);a_min=a_max;}
    
    return ((a_min+a_max)/2);

}

 
//Итерационный вариант
float pwr(float num, int pow) {
    int i;
    float tmp = 1;
    for (i = 1; i <= pow; i++) {
        tmp *= num;
    }
    return tmp;
}

float fsqrtf(float n)
{
  float f;
  unsigned long *i;

  f = n;
  i = ((unsigned long*)&f);
  *i = (0xbe6f0000 - *i) >> 1;
  n *= f;

  return (n * (1.5 - 0.5 * f * n));
}

 float DBm_to_volt(float a)

{
float x1,x2,y;

y = (a/10);

y =  powf (10,y);

//y =  pwr (10,y);

x1 = (y/1000)*50;

x2 = sqrtf (x1);

//x2 = fsqrtf (x1);
		/*
		 Transf("\r\n");
		 Transf ("DBm в Вольты:");
         sprintf(strng,"%4.2f",x2);
         Transf(strng);
         Transf("\r\n");
		*/
return x2; 

}


 float DBm_to_watt(float a)

{
float x1,y;

y = (a/10);

y =  powf (10,y);

x1 = y/1000;

		 Transf("\r\n");
		 Transf ("DBm в Ватты:");
         sprintf(strng,"%4.2f",x1);
         Transf(strng);
         Transf("\r\n");

return x1; 

}

 float watt_to_v(float a)

{
float x1,x2;

x1 = a*50;

x2 = sqrtf (x1);
		 
 Transf("\r\n");
 Transf ("Вт в Вольты:");
 sprintf(strng,"%4.2f",x2);
 Transf(strng);
 Transf("\r\n");

return x2; 

}
