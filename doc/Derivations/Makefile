FILENAME=kalman
PAPER=$(FILENAME).pdf
TEX_SOURCE=$(FILENAME).tex
BIBLIO=biblio.bib

$(PAPER):$(TEX_SOURCE) $(BIBLIO)
	pdflatex $(TEX_SOURCE)
	pdflatex $(TEX_SOURCE)
	bibtex $(FILENAME)
	pdflatex $(TEX_SOURCE)
	pdflatex $(TEX_SOURCE)


all:$(PAPER)

view:$(PAPER)
	evince $(PAPER)&

clean:
	@rm -f *~ *.aux *.bbl *.blg *.log *.out *.toc