function [noise] = getNoise()
    % ---------------- NOISE SELECTION ----------------
    noiseOptions = {'Noisy', 'Noise-free'};
    noiseIndex = centeredMenu('Noisy or noise-free scenario?', noiseOptions);
    noise = noiseOptions{noiseIndex};
end

