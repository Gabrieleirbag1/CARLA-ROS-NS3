path_loss_exponent=2.4
reference_loss=46;
transmission_power=16.0206;
fading_mean=0;
fading_var=32;

cd ~/ns-allinone-3.30.1/ns-3.30.1/
 ./waf --run "rosns3-example --exp=$path_loss_exponent --ref_loss=$reference_loss --tx_power=$transmission_power --mean=$fading_mean --var=$fading_var"