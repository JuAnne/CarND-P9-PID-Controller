import sys
import re

fptr = open( sys.argv[1], 'rt' )

cte = 0.0
steer = 0.0

while( True ) :
  tmp = fptr.readline()
  if ( tmp == '' ) :
    break
  tmp = tmp.rstrip( "\r\n" )
  m = re.search( "p_error: ", tmp )
  if ( m != None ) :
    m = re.split( " ", tmp )
    p_error = m[1]
    i_error = m[3]
    d_error = m[5]
    print(p_error + " " + i_error + " " + d_error)  
"""  
  m = re.search( "CTE: ", tmp )
  if ( m != None ) :
    m = re.split( " ", tmp )
    cte = m[1]
    steer = m[4]
    print(cte + " " + steer)
"""
fptr.close()