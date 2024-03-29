idf_component_register(	
	SRCS 		   "bdc_motor_mcpwm_impl.c"	"bdc_motor.c"
	INCLUDE_DIRS   "include1"
    PRIV_REQUIRES "driver" "freertos"
)
