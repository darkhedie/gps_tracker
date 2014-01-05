/*
 * SIRF.h
 *
 *  Created on: 30.09.2011
 *      Author: darkhedie
 */

#ifndef SIRF_H_
#define SIRF_H_

struct ID_02	{
	unsigned int  	Length;
	unsigned char	Msg_ID;
			 long	X_Pos;
			 long 	Y_Pos;
			 long 	Z_Pos;
	    	 int 	X_Speed;
	    	 int 	Y_Speed;
	    	 int 	Z_Speed;
	unsigned char 	Mode_1;
	unsigned char 	HDOP;
	unsigned char	Mode_2;
	unsigned int	GPS_Week;
	unsigned long	GPS_Tow;
	unsigned char	SV_In_Fix;
	unsigned char	CH1_PRN;
	unsigned char	CH2_PRN;
	unsigned char	CH3_PRN;
	unsigned char	CH4_PRN;
	unsigned char	CH5_PRN;
	unsigned char	CH6_PRN;
	unsigned char	CH7_PRN;
	unsigned char	CH8_PRN;
	unsigned char	CH9_PRN;
	unsigned char	CH10_PRN;
	unsigned char	CH11_PRN;
	unsigned char	CH12_PRN;
	unsigned int	Checksum;
};

struct ID_41	{
	unsigned int  	Length;
	unsigned char	Msg_ID;
	unsigned int	Nav_Valid;
	unsigned int	Nav_Type;
	unsigned int	Week_Number;
	unsigned long	TOW;
	unsigned int	Year;
	unsigned char	Month;
	unsigned char	Day;
	unsigned char	Hour;
	unsigned char 	Minute;
	unsigned int	Second;
	unsigned long	Sat_ID_List;
	unsigned long	Latitude;
	unsigned long 	Longtitude;
	unsigned long	Altitude;
	unsigned long 	Altitude_MSL;
    unsigned char	Map_Datum;
    unsigned int	Speed_Over_Ground;
    unsigned int	Course_Over_Ground;
    unsigned int	Magnetic_Variation;
    unsigned int	Climb_Rate;
    unsigned int	Heading_Rate;
    unsigned long	EHPE;
    unsigned long	EVPE;
    unsigned long	ETE;
    unsigned int	EHVE;
    unsigned long	Clock_Bias;
    unsigned long	Clock_Bias_Error;
    unsigned long	Clock_Drift;
    unsigned long	Clock_Drift_Error;
    unsigned long	Distance;
    unsigned int	Distance_Error;
    unsigned int	Heading_Error;
    unsigned char	SV_In_Fix;
    unsigned char	HDOP;
    unsigned char	ModeInfo;
    unsigned int	Checksum;
};


#endif /* SIRF_H_ */
