idf_component_register(	
	SRCS 		   "detect.c"	
	INCLUDE_DIRS   "include"
	REQUIRES	   bdc_motor
	PRIV_REQUIRES	"driver" "freertos"
)
