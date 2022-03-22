function sha1 = getVariableChecksum(v)
%GETVARIABLECHECKSUM Calculate the checksum over a variable.
%   see:
%   https://stackoverflow.com/questions/44687495/matlab-create-md5-checksum-for-variables
%

    % Get a bytestream from the input. Note that this calls saveobj.
    inbs = getByteStreamFromArray(v);

    % Create hash using Java Security Message Digest.
    md = java.security.MessageDigest.getInstance('SHA1');
    md.update(inbs);

    % Convert to uint8.
    d = typecast(md.digest, 'uint8');

    % Convert to a hex string.
    sha1 = dec2hex(d)';
    sha1 = lower(sha1(:)');

end
