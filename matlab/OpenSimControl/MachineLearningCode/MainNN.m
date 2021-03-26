


% Buscar el path donde se ubica el programa
% Los conjuntos de datos deben estar en la misma carpeta

file_path = which(mfilename);
id_ch_folders = find(file_path =='\');
pathTraining = file_path(1:id_ch_folders(end-2));
disp(['Using training path: ', pathTraining]);



MultNN = FALSE; 
EnableBayesOpt = FALSE; % Activar o no la optimizacion Bayesiana
g_labels= {'0'; '1'; '1.1'; '2'; '3'; '4'}; % Valores de la escala de Ashworth



prompt = 'Cuantas muestras de entrenamiento se van a utilizar? ';
NNinputLayer = input(prompt);

x_train = cell(NNinputLayer,1); % Es recomendable ocupar el espacio de memoria que se va a usar pre definiendo las variables
y_train = cell(NNinputLayer,1);
X = cell(pathTrainingData,1);


%programa para importar la data resultada de la simulacion de Opensim
ImportTrainingData();


%Division de conjuntos de datos para el aprendizaje
[X_training,Y_training,X_test,Y_test,X_validation,Y_validation] = SelecTargetData(X,Labels);

%Ordenar la data de mayor a menor, para que asi el algoritmo coja conjuntos
%enteros de la misma, y no haya tanta perdida de informacion
sequenceLengths = cellfun(@(X) size(X,2), X_training);
[sequenceLengthsSorted,idx] = sort(sequenceLengths);
X_training = X_training(idx);
Y_training = Y_training(idx);

figure
bar(sequenceLengthsSorted)
xlabel("Sequence")
ylabel("Length")
title("Sorted Data")


%observar comportamiento de la data en frecuencia, como posible extracción
%de caracteristicas para el algoritmo
 fs = 300; 
figure , subplot(6,1,1); pspectrum(x_train1{1,1},fs,'spectrogram','TimeResolution',0.5), title('G0') ; subplot(6,1,2); pspectrum(x_train1{2,1},fs,'spectrogram','TimeResolution',0.5), title('G1'); subplot(6,1,3); pspectrum(x_train1{3,1},fs,'spectrogram','TimeResolution',0.5) ,title('G1.1') ;subplot(6,1,4); pspectrum(x_train1{4,1},fs,'spectrogram','TimeResolution',0.5) ,title('G2'); subplot(6,1,5); pspectrum(x_train1{5,1},fs,'spectrogram','TimeResolution',0.5), title('G3') ;subplot(6,1,6); pspectrum(x_train1{6,1},fs,'spectrogram','TimeResolution',0.5), title('G4')
[instFreqG0,t0] = instfreq(x_train1{1,1},fs); [instFreqG1,t1] = instfreq(x_train1{2,1},fs); [instFreqG11,t11] = instfreq(x_train1{3,1},fs); [instFreqG2,t2] = instfreq(x_train1{4,1},fs);[instFreqG3,t3] = instfreq(x_train1{5,1},fs); [instFreqG4,t4] = instfreq(x_train1{6,1},fs); figure, subplot(6,1,1); plot(t0,instFreqG0) ,title('G0'), xlabel('Time (s)'), ylabel('Instantaneous Frequency') , subplot(6,1,2); plot(t1,instFreqG1) ,title('G1') ,xlabel('Time (s)'), ylabel('Instantaneous Frequency'), subplot(6,1,3); plot(t11,instFreqG11) ,title('G1.1'), xlabel('Time (s)'), ylabel('Instantaneous Frequency') , subplot(6,1,4); plot(t2,instFreqG2) ,title('G2') ,xlabel('Time (s)'), ylabel('Instantaneous Frequency'), subplot(6,1,5); plot(t3,instFreqG3) ,title('G3'), xlabel('Time (s)'), ylabel('Instantaneous Frequency') , subplot(6,1,6); plot(t4,instFreqG4) ,title('G4') ,xlabel('Time (s)'), ylabel('Instantaneous Frequency')
[pentropyG0,tg0] = pentropy(x_train1{1,1},fs); [pentropyG1,tg1] = pentropy(x_train1{2,1},fs); [pentropyG11,tg11] = pentropy(x_train1{3,1},fs); [pentropyG2,tg2] = pentropy(x_train1{4,1},fs); [pentropyG3,tg3] = pentropy(x_train1{5,1},fs); [pentropyG4,tg4] = pentropy(x_train1{6,1},fs);  figure,  subplot(6,1,1), plot(tg0,pentropyG0), title('G0'), ylabel('Spectral Entropy') , subplot(6,1,2), plot(tg1,pentropyG1), title('G1'), xlabel('Time (s)'), ylabel('Spectral Entropy'),  subplot(6,1,3), plot(tg11,pentropyG11), title('G1.1'), ylabel('Spectral Entropy') , subplot(6,1,4), plot(tg2,pentropyG2), title('G2'), xlabel('Time (s)'), ylabel('Spectral Entropy'),  subplot(6,1,5), plot(tg3,pentropyG3), title('G3'), ylabel('Spectral Entropy') , subplot(6,1,6), plot(tg4,pentropyG4), title('G4'), xlabel('Time (s)'), ylabel('Spectral Entropy')

%Transformar los conjuntos de datos a frecuencia instantanea como posible
%entrada al algoritmo
instfreqTrain = cellfun(@(x)instfreq(x,fs)',X_training,'UniformOutput',false);
instfreqTest = cellfun(@(x)instfreq(x,fs)',X_test,'UniformOutput',false);
instfreqValidation = cellfun(@(x)instfreq(x,fs)',X_validation,'UniformOutput',false);


%Definicion Capas del algoritmo
 layers = [ ...
    sequenceInputLayer(6)
    bilstmLayer(38,'OutputMode','last')
    dropoutLayer(0.7)
    fullyConnectedLayer(6)
    softmaxLayer
    classificationLayer
    ];

%opciones de hiperparametros para el algoritmo
options = trainingOptions('adam', ...
    'ExecutionEnvironment','auto', ...
    'GradientThreshold',1, ...
    'MaxEpochs',500, ...
    'MiniBatchSize',10, ...
    'ValidationData',{instfreqValidation,Y_validation},...
    'ValidationFrequency',10,...
    'L2Regularization',0.000000001,...
    'SequenceLength','longest', ...
    'Shuffle','every', ...
    'plots','training-progress', ...
    'Verbose',0);

%Entrenar el algoritmo
net = trainNetwork(X_training,Y_training,layers,options);


%Clasificar el conjunto de datos de entrenamiento
 YPredTraining = classify(net,X_training, ...
    'MiniBatchSize',10, ...
    'SequenceLength','longest');

%Desempeno del algoritmo en el conjutno de entrenamiento
LSTMAccuracyTraining = sum(YPredTraining == Y_training)/numel(Y_training)*100;
%Matriz de confusion para el conjunto de datos de entrenamiento
figure
ccLSTM = confusionchart(Y_training,YPredTraining);
ccLSTM.Title = 'Confusion Chart for LSTM';
ccLSTM.ColumnSummary = 'column-normalized';
ccLSTM.RowSummary = 'row-normalized';

%Clasificar el conjunto de datos de prueba
 YPredTest = classify(net,X_test, ...
    'MiniBatchSize',10, ...
    'SequenceLength','longest');

%Desempeno del algoritmo en el conjutno de prueba
LSTMAccuracyTest = sum(YPredTest == Y_test)/numel(Y_test)*100;
%Matriz de confusion para el conjunto de datos de prueba
figure
ccLSTM = confusionchart(Y_test,YPredTest);
ccLSTM.Title = 'Confusion Chart for LSTM';
ccLSTM.ColumnSummary = 'column-normalized';
ccLSTM.RowSummary = 'row-normalized';



%Optimizacion Bayesiana
if EnableBayesOpt == TRUE 


%Variables a optimizar
optimVars = [
optimizableVariable('LayerSize',[50 100],'Type','integer')
optimizableVariable('DropRegularization',[1e-1 1],'Transform','log')
optimizableVariable('L2Regularization',[1e-10 1e-2],'Transform','log')];

ObjFcn = makeObjFcn(X_training,Y_training,X_validation,Y_validation);

BayesObject = bayesopt(ObjFcn,optimVars, ...
'MaxTime',14*60*60, ...
'IsObjectiveDeterministic',false, ...
'UseParallel',false);
end





%Multiple Neural Network training
  %implica entrenar un conjutno de redes neuronales y sacar la media de las
  %predicciones como resultado final
  
if MultNN == TRUE  
  numNN=10;
  nets = cell(1, numNN);
  for w=1:numNN
      
   nets{w} = trainNetwork(X_training,Y_training,layers,options);
      
  end
          
  Y_total=0;     
  for i=1:numNN     
      
  net = nets{i};     
  Ypredict = net.predict(Test_X);
  Y_total = Y_total + Ypredict; 
  
  end
  
  Y_ave = Y_total/numNN;
  
  Y_Pred = cell(numel(Test_X),1);
  for h=1:numel(Test_Y)
   
   [Valmax, id] = max(Y_ave(h,:));   
   Y_Pred{h} = g_labels{id};  
      
  end

  Y_Pred = categorical(Y_Pred);

LSTMAccuracyMulti = sum(Y_Pred == Y_test)/numel(Y_test)*100;
  
end