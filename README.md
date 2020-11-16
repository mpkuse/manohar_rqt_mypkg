https://wiki.ros.org/rqt

a sample rqt plugin.

## A custom .ui file
Use Qt Designer to create a GUI. Be sure to have it of the type QWidget.
You can have a look at the .ui file (an XML) to know the type. If it is QDialog for example, it will
not work with rqt.


## SIGNALS and SLOTS
It is easy to setup signals and slots. You can set the .connect() in the constructor.
You need to correctly set the names of the buttons in the Qt-designer. Using qt-docs
you can know the signals available for the datatype. 
