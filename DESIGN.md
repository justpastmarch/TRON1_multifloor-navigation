# TRON1 Diagnostic Visualization Design System

## 1. Atmosphere & Identity

An engineering control-room surface: dark, legible, and explicit about evidence. The signature is the seven-stage stair pipeline, where color communicates completion, the active gate, and the exact fault location without decorative motion.

## 2. Color

| Role | Token | BGR | Usage |
|---|---|---:|---|
| Canvas | `canvas` | `(24, 21, 18)` | Full image background |
| Panel | `panel` | `(38, 34, 30)` | Evidence and metric regions |
| Border | `border` | `(80, 75, 68)` | Structural separation |
| Text primary | `text_primary` | `(238, 238, 232)` | State and phase labels |
| Text secondary | `text_secondary` | `(165, 166, 160)` | Metadata and details |
| Active | `active` | `(255, 196, 72)` | Currently evaluated phase |
| Complete | `complete` | `(104, 210, 124)` | Confirmed phases |
| Fault | `fault` | `(82, 82, 235)` | Terminal rejection |
| Pending | `pending` | `(118, 112, 104)` | Unevaluated phases |

Only semantic status colors are saturated. Recorded values and timestamps never imply success by color alone.

## 3. Typography

- OpenCV `FONT_HERSHEY_DUPLEX` for headings and active state.
- OpenCV `FONT_HERSHEY_SIMPLEX` for metadata and evidence.
- Minimum rendered text height is approximately 14 px at the native 1280×720 panel size.
- Uppercase is reserved for machine states and phase identifiers.

## 4. Spacing & Layout

- Base unit: 8 px.
- Native canvas: 1280×720.
- Header: 96 px; state pipeline: 300 px; evidence panel: remaining height.
- The seven phases use a 4+3 grid so long identifiers remain readable.
- All primary information stays visible without scrolling in `rqt_image_view`.

## 5. Components

### Phase Node
- **Structure**: phase index, wrapped identifier, semantic border/fill.
- **Variants**: pending, active, complete, fault.
- **States**: static; state changes are communicated by color and label only.
- **Accessibility**: color is reinforced by `WAIT`, `ACTIVE`, `DONE`, or `FAULT` text.

### Evidence Panel
- **Structure**: outcome, bag/profile metadata, elapsed/sample metrics, progress bar, latest reason.
- **Variants**: running, succeeded, faulted, incomplete.
- **Accessibility**: fault and outcome are always rendered as text, not color alone.
- **Direction**: progress uses a centered zero marker; reverse evidence fills left in fault red and target progress fills right.

## 6. Motion & Interaction

- No decorative animation.
- Frames update only when replay evidence changes or at a bounded preview cadence.
- `q` and `Esc` close the local preview; headless mode always remains available.

## 7. Depth & Surface

- Strategy: tonal shift plus one-pixel borders.
- No shadows or gradients; this is a diagnostic instrument, not a marketing surface.
- The active phase receives the strongest border contrast.

## 8. Accessibility Constraints & Accepted Debt

- Text/state redundancy prevents color-only interpretation.
- The 1280×720 source remains readable when fitted to a 16:9 viewer.
- Accepted debt: OpenCV built-in fonts do not render Korean; runtime labels remain concise English identifiers while operator documentation is Korean.
