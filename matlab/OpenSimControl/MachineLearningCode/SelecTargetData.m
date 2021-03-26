function [X_training,Y_training,X_test,Y_test,X_validation,Y_validation] = SelecTargetData(X,Labels)
%Funcion que divide los conjuntos de datos para el entrenamiento, prueba y
%validacion, para ser usados correctamente en el proceso de aprendizaje

ng_Ashworth =6;
qtyperG = numel(Labels)/ng_Ashworth;

%Seleccionar las muestras de cada ganancia
g0X = X(Labels=='0'); g0Y = Labels(Labels=='0');
g1X = X(Labels=='1'); g1Y = Labels(Labels=='1');
g1_1X = X(Labels=='1.1'); g1_1Y = Labels(Labels=='1.1');
g2X = X(Labels=='2'); g2Y = Labels(Labels=='2');
g3X = X(Labels=='3'); g3Y = Labels(Labels=='3');
g4X = X(Labels=='4'); g4Y = Labels(Labels=='4');

%Dividir en partes iguales para el entranmiento y test, con 90% training y
% 10% test
[trainG0,~,testG0] = dividerand(qtyperG,0.9,0.0,0.1);
[trainG1,~,testG1] = dividerand(qtyperG,0.9,0.0,0.1);
[trainG1_1,~,testG1_1] = dividerand(qtyperG,0.9,0.0,0.1);
[trainG2,~,testG2] = dividerand(qtyperG,0.9,0.0,0.1);
[trainG3,~,testG3] = dividerand(qtyperG,0.9,0.0,0.1);
[trainG4,~,testG4] = dividerand(qtyperG,0.9,0.0,0.1);

%Buscar las variables definidas para cada set
XTrainG0 = g0X(trainG0); YTrainG0 = g0Y(trainG0);
XTrainG1 = g1X(trainG1); YTrainG1 = g1Y(trainG1);
XTrainG1_1 = g1_1X(trainG1_1); YTrainG1_1 = g1_1Y(trainG1_1);
XTrainG2 = g2X(trainG2); YTrainG2 = g2Y(trainG2);
XTrainG3 = g3X(trainG3); YTrainG3 = g3Y(trainG3);
XTrainG4 = g4X(trainG4); YTrainG4 = g4Y(trainG4);
XTestG0 = g0X(testG0); YTestG0 = g0Y(testG0);
XTestG1 = g1X(testG1); YTestG1 = g1Y(testG1);
XTestG1_1 = g1_1X(testG1_1); YTestG1_1 = g1_1Y(testG1_1);
XTestG2 = g2X(testG2); YTestG2 = g2Y(testG2);
XTestG3 = g3X(testG3); YTestG3 = g3Y(testG3);
XTestG4 = g4X(testG4); YTestG4 = g4Y(testG4);

%Training Set
X_training = [XTrainG0; XTrainG1; XTrainG1_1; XTrainG2; XTrainG3; XTrainG4];
Y_training = [YTrainG0; YTrainG1; YTrainG1_1; YTrainG2; YTrainG3; YTrainG4];

%Test Set
X_test = [XTestG0; XTestG1; XTestG1_1; XTestG2; XTestG3; XTestG4];
Y_test = [YTestG0; YTestG1; YTestG1_1; YTestG2; YTestG3; YTestG4];

%Validation Set
%Se eligen de manera aleatoria, mismo tamano que el test de prueba, 10%
idx = randperm(size(X_training,1),numel(X_test));
X_validation = X_training(idx);
X_training(idx) = [];
Y_validation = Y_training(idx);
Y_training(idx) = [];


end

