TODO: Add coding convention
	- contiki style
	- p_ prefix for pointer variables
	- len_ prefix for length of other vars

TODO:
	- driver can be used through structs, aim of simplyfiying interaction
	- total control still available through the basic dw_read_reg, dw_write_reg.

TODO:
	- Entry point, dw1000_base_driver?
	- dw_transmit
	- dw_receive
	- dw-read/write reg < add group doxygen
		* registers
		* subregisters
	- opt, group for device communication
		* todo, write
	- add group for
		* ADC, bug
		* Diagnostics, note limitation on fp_power
		* RX/TX
			How to do proper rx/tx with set up conf, commit it to device, call dw_receive/dw_transmit.

TODO:
	Interrupt system

TODO:
	describe
		Register address
		register length
		bitfields
		bitfiled_mask

TODO:
	organisation
		- HOST
		- DW1000
		- interrupt geneartion

	base.h defines organistaion. add to group?