function A = simple_read_ply(Path)

[fid,Msg] = fopen(Path,'rt');	% open file in read text mode
if fid == -1, error(Msg); end

Buf = fscanf(fid,'%s',1);
if ~strcmp(Buf,'ply')
   fclose(fid);
   error('Not a PLY file.'); 
end


%% read headers
NumProperties = 0;
PropertyNames = [];		% structure of lists of property names
LengthHeader = 0;
isEndheader = 0;

while(1)
    
    Buf = fgetl(fid);   								% read one line from file
    BufRem = Buf;
    Token = {};

    Count = 0;
    LengthHeader = LengthHeader + 1;
    while ~isempty(BufRem)								% split line into tokens
      [tmp,BufRem] = strtok(BufRem);
      if ~isempty(tmp)
         Count = Count + 1;							% count tokens
         Token{Count} = tmp;
      end
    end
   
    if Count        
      switch lower(Token{1})
      case 'format'		% read data format
         if Count >= 2
            Format = lower(Token{2});
            
            if Count == 3 & ~strcmp(Token{3},'1.0')
               fclose(fid);
               error('Only PLY format version 1.0 supported.');
            end
         end
      case 'element'		% element name
         if Count >= 3            
            CurElement = Token{2};
            ElementCount = str2double(Token{3});
         end         
      case 'property'	% element property
         if ~isempty(CurElement) & Count >= 3            
            NumProperties = NumProperties + 1;
         end         
      case 'end_header'	% end of header, break from while loop
          break;		
      end
    end          
end


%% read elements
A = fscanf(fid, '%f');
A = (reshape(A, NumProperties, ElementCount))';
fclose(fid);

end