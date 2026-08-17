 #!/bin/sh
function generate_options()
{ 
  use_authorized_state=$1
  cd $(dirname $0)
  sed -e s/USEAUTHORIZAED/${use_authorized_state}/g -e  s/MYNAMESPACE/$2/g $(cd $(dirname $0); pwd)/global_var.py.tem > src/master_node/global_var.py
  cd -
}

generate_options $@ 

