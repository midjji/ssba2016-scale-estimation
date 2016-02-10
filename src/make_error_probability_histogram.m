function [bin_edges, values] = make_error_probability_histogram(s_nums, s_dens, s_gt, foo, e_thr)

e = abs(s_nums ./ s_dens ./ s_gt - 1);

figure(1000)
h = histogram(foo, 100);
bin_edges = h.BinEdges;	
close(1000)	

inliers = zeros(1,length(bin_edges));
total = zeros(1,length(bin_edges));

for k = 1:length(e)
	[~, bkt] = min(abs(bin_edges - foo(k)));
	if (e(k) < 0.1)
		inliers(bkt) = inliers(bkt) + 1;
	end
	total(bkt) = total(bkt) + 1;
end

values = inliers ./ total;