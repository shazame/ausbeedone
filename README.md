Ausbee'Done
===========

Fichiers sources de la stratégie de l'équipe Ausbee'Done pour la Coupe de
France de Robotique, depuis l'année 2014.

Ce projet est basé sur l'architecture *AUSBEE* et utilise la bibliothèque
*libausbee* que nous avons conçu et dont les sources sont disponibles ici :
https://github.com/Kev-J/ausbee

Setting up
----------

Firstly you should get *AUSBEE* source files. Use your favorite shell to `cd`
into the desired directory and run:

`git clone https://github.com/Kev-J/ausbee.git`

Then you need to configure your board, see the instructions in
`ausbee/README`. Be careful to add `-lm` as linker flag (Toolchain) to use the
math library.

Once this is done you can get this project's source files:

`cd ausbee/Project`

`git clone https://github.com/shazame/ausbeedone.git`

If, like us, your want to use FreeRTOS, you should add this line to the file
`project.mk`:

`PROJECT_INCLUDES += $(FREERTOS_INCLUDES_DIR)`

That's all!

Compiling
---------

`cd ausbeedone && make`

This will download configured dependencies, such as FreeRTOS.

Sending the code on the board
-----------------------------

`make program`
