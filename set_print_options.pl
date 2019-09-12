/* For SWI Prolog
  Change the length of lists that are fully displayed.
  Default is 10, which displays up to 9 items, and
  the rest are displayed as "|...".
  
  Note: this option also affects the depth of nesting that is printed.
*/ 

:- set_prolog_flag( answer_write_options
                  , [quoted(true),portray(true),max_depth(100)]).

/* To see the current print options the following query can
   be made.  The result shown is the default.
   
   ?- current_prolog_flag(answer_write_options,Flags).
   Flags = [quoted(true),portray(true),max_depth(10)]
   
*/
