#/bin/bash

cte_w=115.1
epsi_w=95.1
v_w=0.007
dlt_w=95.1
a_w=1.1
dltd_w=20.1
ad_w=1.1

make && ./mpc $cte_w $epsi_w $v_w $dlt_w $a_w $dltd_w $ad_w
