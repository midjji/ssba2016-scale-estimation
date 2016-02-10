load('scaleest_results2_real_e.mat');
%%
%#ok<*AGROW>

k = 1101;

f = 1;
close all;

figure(9);clf
plot_legend = {};
ii = 1;

sigma_n = 0;
for sigma_n = [0 0.5 1.0 1.5 2]

	nscales = [];
	for k = 1:length(test)
		tt = test{k};
		
		rho = 0.0;

		if (tt.rho == rho) && (tt.sigma_n == sigma_n)
			nscales = [nscales; tt.scale_nums ./ tt.scale_dens / tt.gt_scale;];
		end
	end
	
	figure(1000); clf; hold on
	h = histogram(nscales, -5:0.01:5, 'normalization', 'probability');
	BinEdges = h.BinEdges;
	Values = h.Values;
	close(1000);
	
	figure(9);
	semilogy(BinEdges(1:end-1), Values+ 1e-10, 'LineWidth', 1.5); hold on
	
	plot_legend{ii} = sprintf('\\sigma_n = %.1f', sigma_n);
	xlim([0.5 1.5]);
	ylim([5e-4 1]);
	
	ii = ii + 1;
end

figure(9);
legend(plot_legend, 'Location', 'northwest');
xlabel('normalized scale $$\hat{s} / s_{\mathrm{gt}}$$', 'Interpreter', 'Latex');
ylabel('$$\\log(p(\hat{s} / s_{\mathrm{gt}}$$))', 'Interpreter', 'Latex');

%%
%close all;

figure(10);clf
plot_legend = {};
ii = 1;

e_thr = 0.1;

for sigma_n = [0 0.5 1.0 1.5 2]

	s_gt = [];
	s_nums = [];
	s_dens = [];
	s_iratios = [];
	nscales = [];
	for k = 1:length(test)
		tt = test{k};
		
		rho = 0.0;

		if (tt.rho == rho) && (tt.sigma_n == sigma_n)
			s_nums = [s_nums; tt.scale_nums];
			s_dens = [s_dens; tt.scale_dens];
			s_iratios = [s_iratios; tt.iratios];
			s_gt = [s_gt; tt.gt_scale * ones(size(tt.scale_nums))];
		end
	end

	[num_edges, num_prob_e] = make_error_probability_histogram(s_nums, s_dens, s_gt, abs(s_nums), e_thr);
	[den_edges, den_prob_e] = make_error_probability_histogram(s_nums, s_dens, s_gt, abs(s_dens), e_thr);
	
	figure(10);
	plot(num_edges, num_prob_e, 'LineWidth', 1.5); hold on
	ylim([0,1.02]);
	
	figure(11);
	plot(den_edges, den_prob_e, 'LineWidth', 1.5); hold on
	ylim([0,1.02]);
	
	plot_legend{ii} = sprintf('\\sigma_n = %.1f', sigma_n);
	
	ii = ii + 1;
end

figure(10);
legend(plot_legend, 'Location', 'southeast');
xlabel('$$\mathrm{abs}(s_{\mathrm{numerator}})$$', 'Interpreter', 'Latex');
ylabel(sprintf('$$p(e < %.1f)$$', e_thr), 'Interpreter', 'Latex');

figure(11);
legend(plot_legend, 'Location', 'southeast');
xlabel('$$\mathrm{abs}(s_{\mathrm{denominator}})$$', 'Interpreter', 'Latex');
ylabel(sprintf('$$p(e < %.1f)$$', e_thr), 'Interpreter', 'Latex');
