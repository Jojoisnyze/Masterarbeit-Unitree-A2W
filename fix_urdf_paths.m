file_path = 'z1_description/xacro/z1.urdf';



text_content = fileread(file_path);



% Ersetze 'package://' durch 'z1_description/'



modified_content = strrep(text_content, 'package://z1_description/', 'z1_description/');



fid = fopen(file_path, 'w');



fprintf(fid, '%s', modified_content);



fclose(fid);



disp('Die Pfade in der URDF-Datei wurden erfolgreich angepasst.');