/* Light, dark, or as the system says: the reader's choice, kept for next
   time. Loaded in <head>, so a page opens in its theme rather than
   flashing the other one first. Every [data-theme-toggle] button cycles
   Auto → Light → Dark. */
(() => {
  'use strict';
  const root = document.documentElement;
  const KEY = 'keel-theme';
  const ORDER = ['auto', 'light', 'dark'];
  const NAMES = { auto: 'Auto', light: 'Light', dark: 'Dark' };
  const current = () => root.dataset.theme || 'auto';
  function apply(theme) {
    if (theme === 'light' || theme === 'dark') root.dataset.theme = theme;
    else delete root.dataset.theme;
  }
  try { apply(localStorage.getItem(KEY)); } catch (e) { /* no storage: follow the system */ }

  function label(button) {
    const t = current();
    button.querySelector('.theme-name').textContent = NAMES[t];
    button.setAttribute('aria-label', `Theme: ${NAMES[t]}. Change it`);
  }
  function wire() {
    const buttons = [...document.querySelectorAll('[data-theme-toggle]')];
    for (const b of buttons) {
      label(b);
      b.addEventListener('click', () => {
        const next = ORDER[(ORDER.indexOf(current()) + 1) % ORDER.length];
        apply(next);
        try { localStorage.setItem(KEY, next); } catch (e) { /* kept for this visit only */ }
        buttons.forEach(label);
      });
    }
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', wire);
  else wire();
})();
