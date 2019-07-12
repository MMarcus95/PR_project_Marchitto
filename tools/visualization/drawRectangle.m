function drawRectangleInMap(map_, row_, col_,ori_, color_code_,color_code2_)
  hold on;
  rectangle("Position", [col_-1 rows(map_)-row_ 1 1], "FaceColor", color_code_);
  if ori_==1    
    rectangle("Position", [col_ rows(map_)-row_ 1 1], "FaceColor", color_code2_);
    if map_(row_,col_)==1
      disp(row_,col_)
      rectangle("Position", [col_ rows(map_)-row_ 1 1], "FaceColor", "black");
    endif
  elseif ori_==2
    rectangle("Position", [col_-1 rows(map_)-row_+1 1 1], "FaceColor", color_code2_);
  elseif ori_==3
    rectangle("Position", [col_-2 rows(map_)-row_ 1 1], "FaceColor", color_code2_);
  else
    rectangle("Position", [col_-1 rows(map_)-row_-1 1 1], "FaceColor", color_code2_);
  endif    
  hold off;
endfunction

