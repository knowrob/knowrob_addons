
:- module(openease_editor,
    [
        ease_unload_user_package/1,
        ease_load_user_package/1
    ]).

:- dynamic ease_user_term/2.

is_prolog_source_file(_File) :- true.


ease_unload_user_package(PkgDirectory) :-
  atom_concat(PkgDirectory,'/prolog',PrologDir),
  ease_unload_directory(PrologDir).
ease_load_user_package(PkgDirectory) :-
  atom_concat(PkgDirectory,'/prolog',PrologDir),
  ease_load_directory(PrologDir).


ease_unload_directory(Directory) :-
  directory_files(Directory, Entries),
  forall(member(File,Entries),
         ease_unload_file(File)).
ease_load_directory(Directory) :-
  directory_files(Directory, Entries),
  forall((
    member(File,Entries),
    is_prolog_source_file(File)),
    ease_consult(File)).


ease_unload_file(File) :-
  forall(
    ease_user_term(File,Term),
    retractall(:(ease_user,Term))),
  retractall(ease_user_term(File,_)).
ease_consult(File) :-
  open(File, read, Fd),
  read(Fd, First),
  read_data(File, First, Fd),
  close(Fd).

read_data(_, end_of_file, _) :- !.
read_data(File, Term, Fd) :-
  assertz(:(ease_user,Term)),
  assertz(ease_user_term(File,Term)),
  read(Fd, Next),
  read_data(File, Next, Fd).
