Ausbee'Done
===========

Fichiers sources de la stratégie pour la Coupe de France de Robotique 2014 de
l'équipe Ausbee'Done, étudiants en troisième année de l'association Eirbot à
l'ENSEIRB-MATMECA.

Ce projet est basé sur l'architecture *AUSBEE* et utilise la bibliothèque
*libausbee* que nous avons conçu et dont les sources sont disponibles : https://github.com/Kev-J/ausbee

Setting up
----------

Firstly you should get *AUSBEE* source files. Use your favorite shell to `cd`
into the desired directory and run:

`git clone https://github.com/Kev-J/ausbee.git`

Then you need to configure your board, see the instructions in
`ausbee/Software/README`.

Once this is done you can get this project's source files:

`cd ausbee/Software/Project`

`git clone https://github.com/shazame/ausbeedone.git`

That's all!

Compiling
---------

`cd ausbeedone && make`

This will download configured dependencies, such as FreeRTOS.

Sending the code on the board
-----------------------------

`make program`
