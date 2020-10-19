%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function training_t = regulizedFineTuneLiDARTagPoseLieGroup(opt, training_t, P, display)
        
    for i = 1:size(training_t, 2)
        PC = training_t(i).payload_points_h;
        Y = training_t(i).Y_train;
        ideal_frame = training_t(i).ideal_frame;
        box_width = training_t(i).box_width;
        
        if isempty(box_width)
            continue
        end
        
        if isfield(training_t, 'X_train_refined') && ~isempty(training_t(i).X_train_refined)
            X = training_t(i).X_train_refined;
        else
            X = training_t(i).X_train;
        end
        
        if isfield(training_t, 'H_LT_refined') && ~isempty(training_t(i).H_LT_refined)
            H_LT = training_t(i).H_LT_refined;
        else
            H_LT = training_t(i).H_LT;
        end

        [X_train_refined, H_LT_refined] = regulizedFineTuneEachLiDARTagPoseLieGroup(opt, X, Y, PC, H_LT, P, ideal_frame, box_width, display);
        training_t(i).X_train_refined = X_train_refined;
        training_t(i).H_LT_refined = H_LT_refined;
    end
end
