#!/bin/sh

lead='name: icuas_ch'
tail='  - solution:'
sed -e "/$lead/,/$tail/{/$lead/ {p; r session_without_solution.yml
        }; /$tail/p; d }" session.yml > session_updated.yml

lead='  export UAV_NAMESPACE=red;'
tail='windows:'
sed -e "/$lead/,/$tail/{/$lead/ {p; r sc_1.yml
        }; /$tail/p; d }" session_updated.yml > session.yml

./start.sh
sleep 15
mv run.log $1_run_1.log
sleep 10

lead='  export UAV_NAMESPACE=red;'
tail='windows:'
sed -e "/$lead/,/$tail/{/$lead/ {p; r sc_2.yml
        }; /$tail/p; d }" session_updated.yml > session.yml

./start.sh
sleep 15
mv run.log $1_run_2.log
sleep 10
lead='  export UAV_NAMESPACE=red;'
tail='windows:'
sed -e "/$lead/,/$tail/{/$lead/ {p; r sc_3.yml
        }; /$tail/p; d }" session_updated.yml > session.yml

./start.sh
sleep 15
mv run.log $1_run_3.log
sleep 10

lead='  export UAV_NAMESPACE=red;'
tail='windows:'
sed -e "/$lead/,/$tail/{/$lead/ {p; r sc_4.yml
        }; /$tail/p; d }" session_updated.yml > session.yml

./start.sh
sleep 15
mv run.log $1_run_4.log
sleep 10

lead='  export UAV_NAMESPACE=red;'
tail='windows:'
sed -e "/$lead/,/$tail/{/$lead/ {p; r sc_5.yml
        }; /$tail/p; d }" session_updated.yml > session.yml

./start.sh
sleep 15
mv run.log $1_run_5.log
sleep 10

#lead='          x:=-10'
#tail='          x:=-10'
#sed -e "/$lead/,/$tail/{/$lead/ {p; r uav_pos_1.yml
#        }; /$tail/p; d }" session.old.yml
