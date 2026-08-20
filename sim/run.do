if [file exist "work"] {vdel -all}
vlib work

vlog -f files.f  

vsim -voptargs=+acc work.tb_top \

run -all

