idf_component_register(	
	SRCS 		   "detect.c"	
	INCLUDE_DIRS   "include"
	PRIV_REQUIRES	   bdc_motor
	REQUIRES	 "driver" "freertos"
)
