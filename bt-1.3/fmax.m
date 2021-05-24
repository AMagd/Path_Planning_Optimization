function fobj = fmax(fobjarr)
	% -- f = fmax({f1,...,fn})    pointwise maximum
	%
	%    pointwise maximum of n polyhedral functions
	%
	%    corresponds to intersection of epigraphs
	%
	%    Input:
	%      {f1,...,fn}: polyhedral convex functions with same number
	%                   of variables (cell array of polyf objects)
	%    Output:
	%      f: pointwise maximum (polyf object)
	%
	%    see also: fenv, finfc, fsum
	%
	%    for further information, see http://tools.bensolve.org/files/manual.pdf
	
	narginchk(1,1);
	if ~iscell(fobjarr)
		error('invalid argument: cell array expected');
	end
	nn=length(fobjarr);
	for i=1:nn
		if ~isa(fobjarr{i},'polyf')
			error('invalid entry in cell array: polyf object expected');
		end
	end
	if nn>=1
		nvar=fobjarr{1}.nvar;
	else
		error('invalid input');
	end	
	for i=1:nn
		if fobjarr{i}.nvar ~= nvar
			error('functions do not have the same number of variables');
		end
	end
	carr=cell(nn,1);
	for i=1:nn
		carr{i}=fobjarr{i}.epi;
	end
	epi=intsec(carr);
	fobj=polyf(epi);
end