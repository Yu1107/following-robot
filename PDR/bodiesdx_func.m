function [bodiesdx]=bodiesdx_func(Change_person,bodies,oldbodies,oldbodiesdx) %(人數變化,人體骨架偵測,紀錄這次資料成為下一次資料比對,紀錄這次資料成為下一次資料比對)
         %人數變化是增加迴圈  
         if Change_person>0         
                bodies_X=zeros(1,length(bodies) );
                bodies_Y=bodies_X;
                bodies_Z=bodies_X;
                bodies_distance=bodies_X;      
                bodiesdx=zeros(6,1);  
                oldbodiesZEROdx= find(oldbodiesdx~=0);
              %尋找前一次影像與當前影像之人位置 是否一樣(用頭的XYZ比較)
              for i=1:length(oldbodies)
                for j=1:length(bodies)
                    bodies_X(j)= (oldbodies(oldbodiesdx(oldbodiesZEROdx(i))).Position(1,4) -bodies(j).Position(1,4))^2  ;
                    bodies_Y(j)= (oldbodies(oldbodiesdx(oldbodiesZEROdx(i))).Position(2,4) -bodies(j).Position(2,4))^2 ;
                    bodies_Z(j)= (oldbodies(oldbodiesdx(oldbodiesZEROdx(i))).Position(3,4) -bodies(j).Position(3,4))^2 ;
                    bodies_distance(j) = sqrt(  (bodies_X(j)+bodies_Y(j)+bodies_Z(j))  )  ;
                end
                    bodies_distancedx=find(  bodies_distance==min(bodies_distance)  ) ;
                    bodiesdx(oldbodiesZEROdx(i))=bodies_distancedx; 
               end 
               %對於前一次，現在的人是多出來的則填補空缺(不在意編號)
                bodiesDxzero =  find(bodiesdx==0)  ;
                bodiesDxnotzero =  bodiesdx~=0  ;
                bodiesnumber=1:length(bodies);
                bodiesnumber(bodiesdx(bodiesDxnotzero))=[];
                 j=1;
                 for i=    (length(oldbodies)+1)     :   length(bodies)
                     bodiesdx(       bodiesDxzero(j)      )      =    bodiesnumber(j) ;
                    j=j+1;
                end        
         end        
         %人數變化是減少迴圈        
         if Change_person<0              
                bodies_X=zeros(1,length(bodies) );
                bodies_Y=bodies_X;
                bodies_Z=bodies_X;
                bodies_distance=bodies_X;   
                bodiesdx=zeros(6,1);  
                oldbodiesZEROdx= find(oldbodiesdx~=0);   
                %尋找當前影像與前一次影像之人位置 是否一樣(用頭的XYZ比較) 
                %與增加隨圈相比，因為是減少，所以反過來比
                for i=1:length(bodies)            
                    for j=1:length(oldbodies)  
                        bodies_X(j)= (bodies(i).Position(1,4)-oldbodies(oldbodiesdx(oldbodiesZEROdx(j))).Position(1,4) )^2  ;
                        bodies_Y(j)= (bodies(i).Position(2,4)-oldbodies(oldbodiesdx(oldbodiesZEROdx(j))).Position(2,4) )^2 ;
                        bodies_Z(j)= (bodies(i).Position(3,4)-oldbodies(oldbodiesdx(oldbodiesZEROdx(j))).Position(3,4) )^2 ;
                        bodies_distance(j) = sqrt(  (bodies_X(j)+bodies_Y(j)+bodies_Z(j))  )  ;
                    end
                        bodies_distancedx=  bodies_distance==min(bodies_distance)   ;                                   
                        bodiesdx(   oldbodiesZEROdx(bodies_distancedx)  )  =   i ;  
                end                         
         end               
end