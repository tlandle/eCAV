# Paper Writing and Reviewing Guide

General principles distilled from a round of advisor review. Run through this before submitting a draft, and use it as a lens when reviewing your own or someone else's writing. Each item is a principle, a short why, and a self-check question.

## 1. Know the reader

- **Most reviewers stop at the introduction if they do not buy in.** The abstract and intro carry disproportionate weight. Spend time there first.
- **Write for someone with zero context in your subfield.** What is obvious to you in a meeting is not obvious on the page.
- **Read your draft as a skeptical outside reviewer.** Can they parse every sentence on the first pass? If not, rewrite.

## 2. Abstract and introduction

- **Open one level of abstraction higher than feels natural.** Start from the problem a general reader understands, then narrow to your specific contribution.
- **Lead each paragraph with its intent.** Be able to state in one sentence what each paragraph establishes. If you cannot, the paragraph has no job.
- **Define core concepts plainly and early**, before you build on them.
- **Order: gap, then question, then answer.** Make the reader feel the gap before you fill it.

## 3. Terminology and precision

- **Expand every acronym on first use in the body.** The abstract does not count as the definition.
- **Expand it correctly.** Match the standard term, not an approximation.
- **Never invent a compound term.** Use the literature term with a citation, or a plain description. Coined nouns make a reviewer stop and guess.
- **A simple word beats a technical one when they mean the same thing.** Reach for the plain version first.
- **Reserve precise terms for their precise meaning.** Words like "correct", "consistent", "stale", "duplicate" have specific meanings in systems. If you mean one, say that one.
- **Name the version or standard for anything you model or measure.** A protocol or tool without a version invites a question.

## 4. Sentence-level clarity

- **One idea per sentence.** A sentence that needs two reads is two sentences.
- **No dangling pronouns.** Replace "it", "this", "they" with the actual noun whenever the antecedent is even slightly ambiguous.
- **Define a term before you use it.** Never let the reviewer guess what a gate, threshold, or window is.
- **If your advisor cannot parse a sentence, neither can the reviewer.** Rewrite it; do not defend it.

## 5. Structure and flow

- **Definitions come before the things that depend on them.** If you find yourself defining a concept after using it, move the definition up.
- **Nothing appears out of nowhere.** Introduce every tool, dataset, model, or metric with one line of context and a citation at first mention.
- **Say each point once.** Repetition reads as padding and breaks cross-references. If a point recurs, state it once and refer back.
- **Set the stage before the details.** Context and system model before prior work and research questions; defer evaluation-specific setup to after the questions.

## 6. Claims and evidence

- **Cite every quantitative claim.** Any number that is not your own measurement needs a source.
- **Every figure and table must be referenced and discussed in the text.** A float nobody talks about looks like filler.
- **Cross-references point to the exact target.** Cite the specific table or figure, not a vague section.
- **Do not over-claim.** Drop hype words. State the mechanism and let the result speak.
- **Do not say what something is not** unless a question demanded the contrast. Negative framing raises doubts nobody had.
- **Never describe measured results as idealized or abstract.** If it is measured, call it measured. Hedging language undercuts real data.

## 7. Figures and tables

- **The caption should make the figure stand alone.** A reader who jumps to the figure should understand it without the body text.
- **The legend must not cover data.** Move it outside the axes or make it multi-column.
- **Reference only items that actually appear in the figure.** A legend that lists symbols not shown confuses more than it helps.
- **Label so the baseline is not misread.** Qualify an "oracle" or "ideal" curve so it is clear why it is not simply the best.
- **Make it impossible to miscount.** If one object over time could be mistaken for many objects, say so in the caption.

## 8. Reviewing a draft (yours or someone else's)

Walk these passes:

- **Per paragraph:** what is its intent? If you cannot state it, the structure is wrong.
- **Per term:** was it defined? was the acronym expanded at first use?
- **Per claim:** is it cited? is the float it refers to actually discussed?
- **Per sentence:** can I parse this in one pass? is there a dangling pronoun? is it two ideas wearing one comma?
- **Per figure:** self-explanatory caption? legend hiding data? honest labels?

## 9. Working with advisor and reviewer comments

- **Keep the comment in place and annotate what you fixed.** Do not silently delete a question; the reviewer wants to see the question and the resolution together.
- **Accept literal strike-through edits silently.** A struck phrase with a replacement is a direct edit, not a discussion.
- **Do not rewrite an anchored paragraph wholesale.** Sidecar comment threads attach to the text; a full rewrite orphans them.

## 10. Length and layout

- **A tall float that can only sit at a column top or bottom leaves whitespace when the text around it runs out.** Before assuming you are over length, relax float placement, resize, or move the float near its reference.
- **Strip review-only markup before a true length check.** Intent tags and inline comments inflate the page count; measure the clean build.
