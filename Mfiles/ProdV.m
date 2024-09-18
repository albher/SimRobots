function vec= ProdV(Wobj, vec)

if nargin== 0
    Wobj= []; vec= [];
elseif nargin==1
    vec=[];
end

[fil, col]= size(vec);

if ~(col== 3 || col==6)
    [~,vec]= Prod(Wobj, vec);
    return
elseif col==3
   vec= [vec, zeros(fil,3)];
end

for i= 1:fil
   [~, vec(i,:)]= Prod(Wobj, vec(i,:));
end

end