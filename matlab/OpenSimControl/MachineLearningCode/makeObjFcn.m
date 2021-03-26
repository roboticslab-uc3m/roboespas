function ObjFcn = makeObjFcn(XTrain,YTrain,XValidation,YValidation)

ObjFcn = @valErrorFun;
    function [valError,cons,fileName] = valErrorFun(optVars)
 
   
    layers = [ ...
                sequenceInputLayer(size(XTrain{1,1},1 ))
                bilstmLayer(optVars.LayerSize,'OutputMode','last')
                dropoutLayer(optVars.DropRegularization)
                fullyConnectedLayer(numel(unique(YTrain)))
                softmaxLayer
                classificationLayer
                ];
    %{      
    layers = [ ...
                sequenceInputLayer(size(XTrain{1,1},1 ))
                lstmLayer(optVars.LayerSize1,'OutputMode','sequence')
                dropoutLayer(0.4)
                lstmLayer(optVars.LayerSize2,'OutputMode','sequence')
                dropoutLayer(optVars.DropRegularization)
                lstmLayer(optVars.LayerSize3,'OutputMode','last')
                fullyConnectedLayer(numel(unique(YTrain)))
                softmaxLayer
                classificationLayer
                ];
        %}  
    options = trainingOptions('adam', ...
                'ExecutionEnvironment','auto', ...
                'MaxEpochs',400, ...
                'MiniBatchSize',20, ...
                'InitialLearnRate',0.01, ...
                'ValidationData',{XValidation,YValidation},...
                'ValidationFrequency',10,...
                'L2Regularization',optVars.L2Regularization,...
                'SequenceLength','longest', ...
                'Shuffle','never', ...
                'Verbose',0, ...
                'Plots','training-progress');    
     
            
      net = trainNetwork(XTrain,YTrain,layers,options);  
      close(findall(groot,'Tag','NNET_CNN_TRAININGPLOT_FIGURE'))
          % Test the Network
          YPredicted = classify(net,XValidation, ...
            'MiniBatchSize',10, ...
            'SequenceLength','longest');
          
        valError = 1 - mean(YPredicted == YValidation);  
        
          fileName = num2str(valError) + ".mat";
          save(strcat('C:\Users\gvalesquez\Documents\MATLAB\TFM gabriel test\BayOpt\',fileName),'net','valError','options')
          cons = [];
 end
 
        
end

