function [DatosMod] = f_ForcesModifications(Datos)
DatosMod=Datos.Data{1, 1}.fuerza(:,1:10:3400);



end

