function [str] = saveAsJson(fname, in)
%SAVEASJSON Simple wrapper so matlabs jsonencode to have it in a more
% human readable style.

% @author Peter Weissig
% @author Sven Lange

% This file is part of
% libmix4sam - Mixtures for Smoothing and Mapping Library
%
% Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
% For more information see https://mytuc.org/mix
%
% libmix4sam is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% libmix4sam is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this software.  If not, see <http://www.gnu.org/licenses/>.
%
% Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)

fid = fopen(fname, 'wt');
str = jsonencode(in);
% str = strrep(str, '{', sprintf('\n{\n'));
% str = strrep(str, '}', sprintf('\n}\n'));
% str = strrep(str, ',', sprintf(',\r'));
fprintf(fid, jsonformatter(str));
fclose(fid);

end

function str = jsonformatter(json_string)
% JSONFORMATTER Converts the string from jsonencode into a better readable
% form.


    % init result & variables
    in_pos  = 0;
    out_pos = 0;
    intent = 0;
    quoted = false;
    str_new = blanks(10000);
    json_length = length(json_string);

    % loop over json-string
    while (in_pos < json_length)
        % select next character
        in_pos = in_pos + 1;

        % copy current symbol
        character = json_string(in_pos);
        out_pos = out_pos + 1;
        str_new(out_pos) = character;

        % check if quotation changes
        if (character == '"')
            quoted = ~quoted;
            continue;
        end

        % check if quoting
        if (quoted)
            continue;
        end

        % check if it was a simple symbol
        if (~any(character == ',[]{}'))
            continue;
        end

        % test for shortcuts ( "[]" or "{}" )
        if (in_pos < json_length)
            if (((character == '[') && (json_string(in_pos+1) == ']')) || ...
                ((character == '{') && (json_string(in_pos+1) == '}')))
                in_pos  = in_pos  + 1;
                out_pos = out_pos + 1;
                str_new(out_pos) = json_string(in_pos);
                continue;
            end
        end

        % check if intentation changes
        if (any(character == '[{'))
            intent = intent + 4;
        elseif (any(character == ']}'))
            intent = intent - 4;
            if (intent < 0)
                warning(['formatting error: negative intentation ', ...
                  ' at character #', num2str(in_pos)]);
                intent = 0;
            end

            % for closing brackets --> brackets after line break
            out_pos = out_pos - 1;
        end


        % insert line break in all cases
        temp = out_pos + 1;
        out_pos = out_pos + 2;
        str_new(temp:out_pos) = '\n';

        temp = out_pos + 1;
        out_pos = out_pos + intent;
        str_new(temp:out_pos) = ' ';

        % check for closing brackets --> brackets after line break
        if (any(character == ']}'))
            out_pos = out_pos + 1;
            str_new(out_pos) = character;
        end
    end

    % add line break in the end
    temp = out_pos + 1;
    out_pos = out_pos + 2;
    str_new(temp:out_pos) = '\n';

    % return result
    str = str_new(1:out_pos);

    % check for errors
    if (quoted)
        warning('formatting error: open quotation in the end');
    end
    if (intent > 0)
        warning(['formatting error: positive intentation ', ...
          ' in the end']);
    end
end
