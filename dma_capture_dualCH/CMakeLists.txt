add_executable(adc_dma_capture
        dma_capture.c
        )
add_library(kiss_fftr kiss_fftr.c)
add_library(kiss_fft kiss_fft.c)
pico_generate_pio_header(adc_dma_capture ${CMAKE_CURRENT_LIST_DIR}/resistor_dac.pio)

target_link_libraries(adc_dma_capture
		pico_stdlib
		hardware_adc
		hardware_dma
		kiss_fftr 
		kiss_fft
		# For the dummy output:
		hardware_pio
		pico_multicore
		)
pico_enable_stdio_usb(adc_dma_capture 1)
pico_enable_stdio_uart(adc_dma_capture 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(adc_dma_capture)

# add url via pico_set_program_url
example_auto_set_url(adc_dma_capture)
