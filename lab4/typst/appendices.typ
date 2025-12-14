// appendix
#let appendices(body) = {
  set heading(numbering: "A.1", supplement: [Appendix])
  counter(heading).update(0)
  show heading.where(level: 1): it => {
    set text(size: 11pt, weight: "medium")
    set align(center)
    show: smallcaps
    v(1em)
    [#it.supplement #counter(heading).display() \ ]
    it.body
  }
  body
}
