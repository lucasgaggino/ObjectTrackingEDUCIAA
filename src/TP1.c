

/*==================[inclusions]=============================================*/


#include <math.h>
#include "TP1.h"       // <= sAPI header




/*==================[macros and definitions]=================================*/


#define TICKRATE_MS 50
#define LED_TOGGLE_MS 1000
/*==================[internal data declaration]==============================*/
unsigned int delayer=0;


/*==================[internal functions declaration]=========================*/
void myTickHook( void *ptr ){
	if(delayer==9){
	 // modificacion para disminuir la frecuencia 10 vecs
		static bool_t ledState = OFF;
		gpioMap_t led = (gpioMap_t)ptr;


		if( ledState ){
			ledState = OFF;
		}
		else{
			ledState = ON;
		}
		gpioWrite( led, ledState );
		delayer=0;
	}
	else{delayer++;}

 }

bool syncWithPC ( tick_t timeout){
	bool patternReceived=FALSE;
	char Patt[]=SYNC_PATTERN;
	uint16_t size=SYNC_PATTERN_SIZE;
	patternReceived=waitForReceiveStringOrTimeoutBlocking(UART_USB,Patt, size, timeout);
	return patternReceived;
}

bool dataFromPC (tick_t timeout, char * data , uint32_t * dataSize){
	char Patt[]=SYNC_PATTERN;
	uint16_t size=SYNC_PATTERN_SIZE;

	return receiveBytesUntilReceiveStringOrTimeoutBlocking (UART_USB, Patt,size, data, dataSize, timeout );
}

int  findDelimiterIndex(char *data,int initIndex){
	int delimIndex=initIndex;

	while( ((char) data[delimIndex]) != '@' && delimIndex<200){
		delimIndex++;
	}
	return delimIndex;
}


bool str2flt(char* str, int init, int end, float* data,int signo){
	float result= 0.0f;
	  size_t dotpos = 0;
	  for (size_t n = init; n < end ; n++)
	  {
	    if (str[n] == '.')
	    {
	      dotpos = (end-init) - (n-init)  - 1;
	    }
	    else
	    {
	      result = result * 10.0f + (str[n]-'0');
	    }
	  }
	  while ( dotpos--)
	  {
	    result /= 10.0f;
	  }
	  (*data)=result*signo;
	return TRUE;
}


bool anglesFromString(char* data, uint32_t  dataSize, struct TargetData * Tdata){
	int i=0;
	int signo=1;
	char car;
	int end;


	for(int iter=0;iter<4;iter++){
		car=(char) data[i];
		while(car != '+' && car != '-' && i != (int) dataSize){
			i++;
			car=data[i];

		}
		if(((char) data[i]) == '-') signo=-1;
		else signo=1;
		i++;
		end=findDelimiterIndex(data,i);
		if(iter==0)
					str2flt(data,i,end,&(Tdata->alfa1),signo);
		if(iter==1)
					str2flt(data,i,end,&(Tdata->beta1),signo);
		if(iter==2)
					str2flt(data,i,end,&(Tdata->alfa2),signo);
		if(iter==3)
					str2flt(data,i,end,&(Tdata->beta2),signo);

		i=end+1;
	}


	return TRUE;

}

void calculateDistance(struct TargetData* Tdata){
	float sh=-0.15;
	float sv=0.01;
	float sz=0.01;

	float a1=PI*(Tdata->alfa1)/180;
	float b1=PI*(Tdata->beta1)/180;
	float a2=PI*(Tdata->alfa2)/180;
	float b2=PI*(Tdata->beta2)/180;

	double A=sin(a1)-sin(a2);
	double B=sin(b1)-sin(b2);
	double C=cos(a1)*cos(b1)-cos(a2)*cos(b2);

	double vecNorm=(A*A+B*B+C*C);

	if(vecNorm!=0){
		(Tdata->lamda)=(sh*A + sv*B + sz*C)/vecNorm;
		if( Tdata->lamda < 0) Tdata->lamda=5;
	}
	else  Tdata->lamda=10;
	Tdata->altura = Tdata->lamda*sin(b1);

}




/*==================[funciones TORRETA]=================================*/

// funciones de fuerzas

//componente x del rozamiento
float Frx(float Vx,float Vy,float Vz,float parametros []){
float cd=parametros[CD_P];
float masa=parametros[MASS_P];
float Vwx=parametros[WINDX_P];
float Vwz=parametros[WINDZ_P];
float radio=parametros[CAL_P]/2;
float da=parametros[ATM_P];

return (cd*da*pi*pow(radio,2)/(2*masa))*sqrt(pow((Vx+Vwx),2)+pow(Vy,2)+pow((Vz+Vwz),2))*(Vx+Vwx);

}

//componente y del rozamiento
float Fry(float Vx,float Vy,float Vz,float parametros []){
float cd=parametros[CD_P];
float masa=parametros[MASS_P];
float Vwx=parametros[WINDX_P];
float Vwz=parametros[WINDZ_P];
float radio=parametros[CAL_P]/2;
float da=parametros[ATM_P];

return (cd*da*pi*pow(radio,2)/(2*masa))*sqrt(pow((Vx+Vwx),2)+pow(Vy,2)+pow((Vz+Vwz),2))*(Vy);

}

//componente z del rozamiento
float Frz(float Vx,float Vy,float Vz,float parametros []){
float cd=parametros[CD_P];
float masa=parametros[MASS_P];
float Vwx=parametros[WINDX_P];
float Vwz=parametros[WINDZ_P];
float radio=parametros[CAL_P]/2;
float da=parametros[ATM_P];

return (cd*da*pi*pow(radio,2)/(2*masa))*sqrt(pow((Vx+Vwx),2)+pow(Vy,2)+pow((Vz+Vwz),2))*(Vz+Vwz);

}

//Sumatorias de fuerzas

//sumatoria para x
float Fx(float Vx,float Vy,float Vz,float parametros[]){
return -Frx(Vx,Vy,Vz,parametros);
}

//sumatoria para z
float Fz(float Vx,float Vy,float Vz,float parametros[]){
return -Frz(Vx,Vy,Vz,parametros);
}

//sumatoria para y
float Fy(float Vx,float Vy,float Vz,float parametros[]){
return -10-Fry(Vx,Vy,Vz,parametros);
}

//estimacion del impacto

/* recive los dos angulos del cañon, los parametros del modelo y un vector de punteros que va a ser donde meta la indormacion que calcule.
 * si no se pudo resolver devuelve un 1, si se pudo resolver devuelve un 0
 */

int calcular_impacto(float alfa,float beta,float parametros[],float *impacto[]){

/*resolucion por runge kutta 4

k1=h*f(xn,yn)
k2=h*f(xn+h/2,yn+k1/2)
k3=h*f(xn+h/2,yn+k2/2)
k4=h*f(xn+h,yn+k3)

yn+1 =yn + 1/6*(k1 +2*k2 + 2*k3 + k4)


seteo del metodo (parametros de la atmosfera y la municion se setean por separado)*/

float h=0.01;            		// paso del metodo
float X=parametros[PIX_P];     // posicion inicial x
float Y=parametros[PIY_P];     // posicion inicial y
float Z=parametros[PIZ_P];     // posicion inicial z
float N=100000;          	 	// cantidad maxima de pasos



float Velociad_objetivo_x = parametros[VTX_P];
float Velociad_objetivo_y = parametros[VTY_P];
float Velociad_objetivo_z = parametros[VTZ_P];

float posx = parametros[POSX_P];
float posy = parametros[POSY_P];
float posz = parametros[POSZ_P];

float V0=parametros[VO_P];

//comienzo a resolver la ecuacion
//declaro las variables que voy a usar

float Vy=V0*sin(2*pi*alfa/360);
float Vx=V0*cos(2*pi*alfa/360)*sin(2*pi*beta/360);
float Vz=V0*cos(2*pi*alfa/360)*cos(2*pi*beta/360);

float Vx_ant=Vx;
float Vy_ant=Vy;
float Vz_ant=Vz;

float T=0;
float i=1;

float  k1x=0;
float  k1y=0;
float  k1z=0;

float  k2x=0;
float  k2y=0;
float  k2z=0;

float  k3x=0;
float  k3y=0;
float  k3z=0;

float  k4x=0;
float  k4y=0;
float  k4z=0;

// resuelvo el sistema
gpioWrite( LED2, ON );

while (Y >= posy && i <= N) {

    i++;
    //cuento el tiempo
    T = T + h;
    gpioWrite( LED3, ON );
    //calculo los k1
    k1x=h*Fx(Vx,Vy,Vz,parametros);
    k1y=h*Fy(Vx,Vy,Vz,parametros);
    k1z=h*Fz(Vx,Vy,Vz,parametros);
    //calculo los k2
    k2x=h*Fx((Vx+k1x/2),(Vy+k1y/2),(Vz+k1z/2),parametros);
    k2y=h*Fy((Vx+k1x/2),(Vy+k1y/2),(Vz+k1z/2),parametros);
    k2z=h*Fz((Vx+k1x/2),(Vy+k1y/2),(Vz+k1z/2),parametros);

    gpioWrite( LED3, OFF );
    //calculo los k3
    k3x=h*Fx((Vx+k2x/2),(Vy+k2y/2),(Vz+k2z/2),parametros);
    k3y=h*Fy((Vx+k2x/2),(Vy+k2y/2),(Vz+k2z/2),parametros);
    k3z=h*Fz((Vx+k2x/2),(Vy+k2y/2),(Vz+k2z/2),parametros);
    //calculo los k4
    k4x=h*Fx((Vx+k3x),(Vy+k3y),(Vz+k3z),parametros);
    k4y=h*Fy((Vx+k3x),(Vy+k3y),(Vz+k3z),parametros);
    k4z=h*Fz((Vx+k3x),(Vy+k3y),(Vz+k3z),parametros);

    //clculo el siguiente paso

    Vx =Vx + (k1x +2*k2x + 2*k3x + k4x)/6;

    Vy =Vy + (k1y +2*k2y + 2*k3y + k4y)/6;

    Vz =Vz + (k1z +2*k2z + 2*k3z + k4z)/6;

    //voy integrando para ir obteniendo las posiciones

    Y=(Vy_ant+Vy)*h/2 + Y;
    X=(Vx_ant+Vx)*h/2 + X;
    Z=(Vz_ant+Vz)*h/2 + Z;

    Vx_ant=Vx;
    Vy_ant=Vy;
    Vz_ant=Vz;

}

//como es MRU puedo calcular con un solo paso la posicion del objetivo

posx=Velociad_objetivo_x*T+posx;
posy=Velociad_objetivo_y*T+posy;
posz=Velociad_objetivo_z*T+posz;

//guardo lo que calcule en el puntero que le pase

*(impacto[X_I])=X;
*(impacto[Y_I])=Y;
*(impacto[Z_I])=Z;
*(impacto[T_I])=T;

*(impacto[XT_I])=posx;
*(impacto[YT_I])=posy;
*(impacto[ZT_I])=posz;

*(impacto[VF_I])=sqrt(Vx*Vx+Vy*Vy+Vz*Vz);
*(impacto[AF_I])=atan2(Vy,sqrt(Vx*Vx+Vz*Vz))*(360/(2*pi));

gpioWrite( LED2, OFF );

if (i==N){
	return 1;
	}
else{
	return 0;
	}

}


//estimacion del angulo para disparo 3D
int apuntar(float *alfa, float *beta,float parametros[], float distancia, float direccion){


	float *impacto[IMP_L];
	float impacto_var[IMP_L];

	impacto[X_I]=&impacto_var[X_I];
	impacto[Y_I]=&impacto_var[Y_I];
	impacto[Z_I]=&impacto_var[Z_I];
	impacto[T_I]=&impacto_var[T_I];

	impacto[XT_I]=&impacto_var[XT_I];
	impacto[YT_I]=&impacto_var[YT_I];
	impacto[ZT_I]=&impacto_var[ZT_I];

	impacto[VF_I]=&impacto_var[VF_I];
	impacto[AF_I]=&impacto_var[AF_I];

	float distancia_impacto=0;
	float angulo_impacto=0;

//parametros del cañon

	float amax_alfa=22;
	float amin_alfa=0;

	float amax_beta=180;
	float amin_beta=-180;



// programa

	int M=100;          //cantidad maxima de iteraciones

	float err=distancia;
	int i=0;

	gpioWrite( LED1, ON );

	while (err >= 0.01*distancia && i<M){



    	*alfa = (amax_alfa+amin_alfa)/2;
    	*beta = (amax_beta+amin_beta)/2;

    	if(calcular_impacto(*alfa,*beta,parametros,impacto)){
    		return 1;
    	}

    	distancia_impacto=sqrt(pow(*impacto[X_I],2)+pow(*impacto[Z_I],2));

    	distancia=sqrt(pow(*impacto[XT_I],2)+pow(*impacto[ZT_I],2));

    	if (distancia_impacto > distancia){
        	amax_alfa=*alfa;
    	}
    	if (distancia_impacto < distancia){
        	amin_alfa=*alfa;
    	}

    	angulo_impacto=atan2(*impacto[X_I],*impacto[Z_I])*(360/(2*pi));
    	direccion=atan2(*impacto[XT_I],*impacto[ZT_I])*(360/(2*pi));


    	if (angulo_impacto > direccion){
        	amax_beta=*beta;
    	}
    	if (angulo_impacto < direccion){
    		amin_beta=*beta;
    	}

    	err=sqrt(pow(*impacto[XT_I]-*impacto[X_I],2) + pow(*impacto[ZT_I]-*impacto[Z_I],2));
    	i++;
    	}

	gpioWrite( LED1, OFF );

	if(i==M){
	return 1;
	}
	return 0;
}

// apuntar con laser
int apuntar_laser(float *alfa, float *beta,float parametros[], float distancia, float direccion){
	*beta=atan2(parametros[POSX_P]-parametros[PIX_P],parametros[POSZ_P]-parametros[PIZ_P])*180/pi;
	*alfa=atan2(parametros[POSY_P]-parametros[PIY_P],distancia-sqrt(pow(parametros[POSX_P]-parametros[PIX_P],2)+pow(parametros[POSZ_P]-parametros[PIZ_P],2)))*180/pi;
	return 0;
}

// contro de motores
// disabe motors
void disable_motors (){
	gpioWrite( EN, DISABLE );
	return;
}
//enable motors
void enable_motors (float * posgiro,float * poselev){

	*posgiro = -90;
	*poselev = -10;

	gpioWrite( EN, ENABLE );

	gpioWrite( DIR_ELEV, DOWN_DIR_ELEV );
	gpioWrite( DIR_GIRO, DOWN_DIR_GIRO );

	while (gpioRead( LIM_GIRO )){
		gpioWrite( S_GIRO, ON );
		delay(DEM_RETURN);
		gpioWrite( S_GIRO, OFF );
		delay(DEM_RETURN);
	}
	while (gpioRead( LIM_ELEV )){
		gpioWrite( S_ELEV, ON );
		delay(DEM_RETURN);
		gpioWrite( S_ELEV, OFF );
		delay(DEM_RETURN);
	}
	return;
}

//mover motores
void mover_motores(float alfa,float beta,float *posgiro,float *poselev){

	float distancia_giro = beta - *posgiro;
	float distancia_elev = alfa - *poselev;
	float sentido_giro = UP_DIR_GIRO;
	float sentido_elev = UP_DIR_ELEV;
	int pasos_elev = 0;
	int pasos_giro = 0;

	*posgiro=*posgiro+distancia_giro;
	*poselev=*poselev+distancia_elev;

	if (distancia_giro < 0){
		distancia_giro = -distancia_giro;
		sentido_giro = DOWN_DIR_GIRO;
	}
	if (distancia_elev < 0){
		distancia_elev = -distancia_elev;
		sentido_elev = DOWN_DIR_ELEV;
	}
	pasos_elev = distancia_elev * STEPS_ELEV / 180;
	pasos_giro = distancia_giro * STEPS_GIRO / 180;

/*	if (beta - *posgiro <0 ){
		*posgiro = *posgiro - pasos_giro*180/STEPS_GIRO;
	}
	else{
		*posgiro = *posgiro + pasos_giro*180/STEPS_GIRO;
	}
	if (alfa - *poselev <0 ){
		*poselev = *poselev - pasos_elev*180/STEPS_ELEV;
	}
	else{
		*poselev = *poselev + pasos_elev*180/STEPS_ELEV;
	}*/

	gpioWrite( DIR_GIRO, sentido_giro );
	gpioWrite( DIR_ELEV, sentido_elev );

	while ((pasos_elev > 0) | (pasos_giro > 0)){
		if (pasos_elev > 0){
			gpioWrite( S_ELEV, ON );
			delay(DEM_WORK);
			gpioWrite( S_ELEV, OFF );
			delay(DEM_WORK);
			pasos_elev =pasos_elev-1;
		}
		if (pasos_giro > 0){
			gpioWrite( S_GIRO, ON );
			delay(DEM_WORK);
			gpioWrite( S_GIRO, OFF );
			delay(DEM_WORK);
			pasos_giro =pasos_giro-1;
		}
	/*	if (!gpioRead( LIM_GIRO )){
			pasos_giro=0;
			*posgiro=-90;
		}
		if (!gpioRead( LIM_ELEV )){
			pasos_elev=0;
			*poselev=0;
		}*/
	}
	return;
}


/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/




int main(void){

   /* ------------- INICIALIZACIONES ------------- */

 /* Inicializar la placa */
 boardConfig();
 /* Inicializar UART_USB a 115200 baudios */
 uartConfig( UART_USB, 115200 );

struct TargetData t_data;
char data[100]="0";
uint32_t dataSize=100;

/*var torretas*/

float alfa=0;
float beta=0;

float posgiro=0;
float poselev=0;

	float distancia=20000;      //distancia al objetivo (en el suelo)(respecto a mi)(max 26km)
	float direccion=90;       //direccion del objetivo (respecto a mi)(entre -180° y 180°)
	float altura=0;

	//parametros del objetivo

		float Velociad_objetivo_x=0;
		float Velociad_objetivo_y=0;
		float Velociad_objetivo_z=0;

		//fuerzas externas

		float Velociad_viento=0;
		float direccion_viento =90;
		float densidad_atmosfera=1.25;

		//parametros de la municion

		float calibre_m=0.38;
		float masa=800;
		float cd=0.295;
		float vel_salida=820;


		//cargo los datos del modelo en el vector de parametros

		float Velocidad_viento_x=Velociad_viento*sin(2*pi*direccion_viento/360);
		float Velocidad_viento_z=Velociad_viento*cos(2*pi*direccion_viento/360);

		float Velocidad_viento_y=0;		//velocidad del viento en y, no lo uso pero lo agrego por si lo quiero poner en el futuro

		float posx=distancia*sin(2*pi*direccion/360);
		float posz=distancia*cos(2*pi*direccion/360);


		float parametros[PAR_L];

		parametros[VTX_P]=Velociad_objetivo_x;
		parametros[VTY_P]=Velociad_objetivo_y;
		parametros[VTZ_P]=Velociad_objetivo_z;
		parametros[POSX_P]=posx;
		parametros[POSY_P]=altura;
		parametros[POSZ_P]=posz;
		parametros[WINDX_P]=Velocidad_viento_x;
		parametros[WINDZ_P]=Velocidad_viento_z;
		parametros[WINDY_P]=Velocidad_viento_y;
		parametros[ATM_P]=densidad_atmosfera;
		parametros[CAL_P]=calibre_m;
		parametros[MASS_P]=masa;
		parametros[CD_P]=cd;
		parametros[VO_P]=vel_salida;

		parametros[PIX_P]=0;			//en estas 3 coordenadas estableces la posicion del cañon respecto al puente
		parametros[PIY_P]=0;
		parametros[PIZ_P]=0;

		gpioConfig( EN, GPIO_OUTPUT );
			gpioConfig( S_ELEV, GPIO_OUTPUT );
			gpioConfig( S_GIRO, GPIO_OUTPUT );
			gpioConfig( DIR_ELEV, GPIO_OUTPUT );
			gpioConfig( DIR_GIRO, GPIO_OUTPUT );
			gpioConfig( LIM_GIRO, GPIO_INPUT );
			gpioConfig( LIM_ELEV, GPIO_INPUT );

			enable_motors (&posgiro,&poselev);

/*==============================================MAIN--LOOP=====================================================*/
	while(1){
	 if(syncWithPC(SYNC_PATTERN_DEFAULT_TIMEOUT)){
		 dataSize=100;
		 if(dataFromPC(DATA_TRANSFER_TIMEOUT,data,&dataSize)){
		 	 anglesFromString(data,dataSize,&t_data);
		 	 calculateDistance(&t_data);
		 	 distancia=t_data.lamda;
		 	 direccion=t_data.alfa1;
		 	 altura=t_data.altura;
		 	//apuntar_laser(&alfa,&beta,parametros,distancia,direccion);
		 	mover_motores(t_data.beta1,-1*t_data.alfa1,&posgiro,&poselev);

		 }

	 }
	 else gpioWrite(LEDR,ON);
	 delay(100);
 }

}
