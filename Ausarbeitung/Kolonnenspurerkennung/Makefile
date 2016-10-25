TEX = thesis.tex

DVI = $(TEX:%.tex=%.dvi)
PS  = $(TEX:%.tex=%.ps)
PDF = $(TEX:%.tex=%.pdf)

dvi: $(DVI)

all: pdf

pdf: $(PDF)

ps:  $(PS)

$(PS): %.ps: %.dvi 
	@echo " *\n * DVIPS: $< > $@ \n *"; \
	dvips -t a4 -o $@ $<

$(DVI): %.dvi: $(TEX) *.tex *.bib
	@echo " *\n * LATEX: $< > $@ \n *"; \
	( \
	latex $<; \
	if grep -q "There were undefined references." $(<:.tex=.log); \
	then \
	  bibtex $(<:.tex=); \
	  latex $<; \
	fi ; \
	if grep -q "There were undefined references." $(<:.tex=.log); \
	then \
	  latex $<; \
	fi ; \
	while grep -q "Rerun to get cross-references right." $(<:.tex=.log); \
	do \
	  latex $<; \
	done; \
	echo "\n\n *******************************************************************************"; \
	echo " *                                                                             *"; \
	echo " *   WARNING SUMMARY                                                           *"; \
	echo " *                                                                             *"; \
	grep "Warning" $(<:.tex=.log); \
	echo " *                                                                             *"; \
	echo " *******************************************************************************\n"; \
	)

$(PDF): %.pdf: %.ps
	@echo " *\n * PS2PDF: $< > $@ \n *"; \
	ps2pdf14 $<

.PHONY: clean
clean:
	@echo " ** Deletion of all backup and automatically generated files.";\
	rm -f *.out *.bbl *.blg *.tpt *.toc *.idx *.log *.aux *.backup *~ $(DVI) $(PS) $(PDF);
